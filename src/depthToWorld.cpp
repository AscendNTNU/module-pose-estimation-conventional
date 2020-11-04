//
// Created by marius on 09.04.2020.
//

#include "depthToWorld.h"




bool distGreaterThan(const Gridpoint& a, const Gridpoint& b) {return a.dist > b.dist;};

void printRot(const tf2::Quaternion& test_rotation) {
    printf("w: %.3f, x: %.3f, y: %.3f, z: %.3f\n", test_rotation.getW(), test_rotation.getX(), test_rotation.getY(), test_rotation.getZ());
}


std::vector<cv::Point3f>
findXYZ(const std::vector<cv::Point2f> &corner_points, const cv::Mat &depth_image, const std::vector<double> &depth_camera_info_K_arr)
{
    std::vector<cv::Point3f> returned_points;

    for (const cv::Point2f & point : corner_points)
    {
        returned_points.emplace_back(findXYZ(point, depth_image, depth_camera_info_K_arr));
    }
    return returned_points;
}

cv::Point3f findXYZ(const cv::Point2f &point, const cv::Mat &depth_image, const std::vector<double> &depth_camera_info_K_arr) {
    const auto& cx = depth_camera_info_K_arr.at(2);
    const auto& cy = depth_camera_info_K_arr.at(5);
    const auto& fx = depth_camera_info_K_arr.at(0);
    const auto& fy = depth_camera_info_K_arr.at(4);


    std::vector<cv::Point3f> valid_close_points = getNCloseValidPoints(point, 6, depth_image);
    // Find the center of these points and the gradient of the fitted plane
    // TODO: Could add a check that no valid close points are at distance greater than set distance threshold in imageCb
    cv::Point3f center; double slope_x, slope_y;
    std::tie(center, slope_x, slope_y) = getPlaneFit(valid_close_points);

    /// The following is the interpolated depth: center.depth + delta_x * (d_depth / d_x) + delta_y * (d_depth / d_y)
    float depth{static_cast<float>(center.z + (point.x - center.x) * slope_x + (point.y - center.y) * slope_y)};

    /// Return the point as an (x, y, z) point
    return cv::Point3f{static_cast<float>((point.x - cx) * depth / fx), static_cast<float>((point.y - cy) * depth / fy), depth};


/*
    float x{0}, y{0}, z{0};
    z = depth_image.at<ushort>(corner_point);
    x = (depth_image.at<ushort>(corner_point.x) - cx) * z / fx;
    y = (depth_image.at<ushort>(corner_point.y) - cy) * z / fy;
    point.x = x;
    point.y = y;
    point.z = z;
*/
};


std::vector<cv::Point2f> getScaledTowardsCenter(const std::vector<cv::Point2f>& points, double scaling) {
    std::vector<cv::Point2f> ret_vector;
    cv::Point2f center = getAverage(points);

    for (const auto& point : points) {
        cv::Point2f diff = point - center;
        ret_vector.emplace_back(point - diff * scaling);
    }
    return ret_vector;

}

std::vector<cv::Point3f> getNCloseValidPoints(const cv::Point2f& point, int n, const cv::Mat& depth_image) {
    // TODO: Optimize: Maybe make this function just look at a few gridpoints close tp the input points, and sort them? Could be quicker...
    std::vector<cv::Point3f> ret_valid_points(n);

    // Initialize i priority_queue with the front being the closest gridpoints to "point"
    std::priority_queue<Gridpoint, std::vector<Gridpoint>, decltype(&distGreaterThan)> closest_points(&distGreaterThan);

    // Initialize the queue with the for gridpoints around "point"
    {
        cv::Point2i p_tmp{static_cast<int>(point.x), static_cast<int>(point.y)};  // Rounding both numbers (down) towards 0
        closest_points.push(Gridpoint{p_tmp, lengthSqr(p_tmp - point), GridpointDirection::x_neg_y_neg});
        p_tmp.x += 1;
        closest_points.push(Gridpoint{p_tmp, lengthSqr(p_tmp - point), GridpointDirection::x_pos_y_neg});
        p_tmp.y += 1;
        closest_points.push(Gridpoint{p_tmp, lengthSqr(p_tmp - point), GridpointDirection::x_pos_y_pos});
        p_tmp.x -= 1;
        closest_points.push(Gridpoint{p_tmp, lengthSqr(p_tmp - point), GridpointDirection::x_neg_y_pos});
    }

    // Get the first n valid points from the queue, adding neighbors as gridpoints are processed
    int idx_valid{0};
    while (idx_valid < n) {
        Gridpoint next_point{closest_points.top()};
        closest_points.pop();

        // If the point is outside the depth_image matrix, skip it
        if (next_point.p.x < 0 || next_point.p.y < 0 || next_point.p.x >= depth_image.cols || next_point.p.y >= depth_image.rows) {
            if (closest_points.empty()) {
                // This error should never be thrown, but might be a side-effect of future changes
                throw std::logic_error("Point is too far outside depth matrix to be computed");
            }
            continue;
        }


        // If the point is valid, add it to the list of valid points and increase the valid points index
        if (depth_image.at<ushort>(next_point.p) != 0) {
            const float depth = depth_image.at<ushort>(next_point.p) * 0.001;  /// Converting from mm to m (always using base si units)
            ret_valid_points.at(idx_valid) = cv::Point3f{static_cast<float>(next_point.p.x),
                                                         static_cast<float>(next_point.p.y),
                                                         depth};
            ++idx_valid;

            if (idx_valid==n) continue;  // Skip adding neighbors if the loop is terminating
        }

        // Add the neighbors to the queue, expanding from the center
        addNeighbors(point, next_point, closest_points);
    }

    return ret_valid_points;
}

void addNeighbors(const cv::Point2f& source_point, const Gridpoint& neighbor_gridpoint,
                  std::priority_queue<Gridpoint, std::vector<Gridpoint>, decltype(&distGreaterThan)>& closest_points_queue) {
    cv::Point2i p_tmp{neighbor_gridpoint.p};

    switch (neighbor_gridpoint.dir) {
        case GridpointDirection::x_neg_y_neg:
            p_tmp.x -= 1;
            closest_points_queue.push(Gridpoint{p_tmp, lengthSqr(p_tmp - source_point), GridpointDirection::x_neg_y_neg});
            p_tmp.x += 1;
            p_tmp.y -= 1;
            closest_points_queue.push(Gridpoint{p_tmp, lengthSqr(p_tmp - source_point), GridpointDirection::y_neg});
            break;

        case GridpointDirection::x_pos_y_neg:
            p_tmp.x += 1;
            closest_points_queue.push(Gridpoint{p_tmp, lengthSqr(p_tmp - source_point), GridpointDirection::x_pos_y_neg});
            p_tmp.x -= 1;
            p_tmp.y -= 1;
            closest_points_queue.push(Gridpoint{p_tmp, lengthSqr(p_tmp - source_point), GridpointDirection::y_neg});
            break;

        case GridpointDirection::x_pos_y_pos:
            p_tmp.x += 1;
            closest_points_queue.push(Gridpoint{p_tmp, lengthSqr(p_tmp - source_point), GridpointDirection::x_pos_y_pos});
            p_tmp.x -= 1;
            p_tmp.y += 1;
            closest_points_queue.push(Gridpoint{p_tmp, lengthSqr(p_tmp - source_point), GridpointDirection::y_pos});
            break;

        case GridpointDirection::x_neg_y_pos:
            p_tmp.x += 1;
            closest_points_queue.push(Gridpoint{p_tmp, lengthSqr(p_tmp - source_point), GridpointDirection::x_neg_y_pos});
            p_tmp.x -= 1;
            p_tmp.y += 1;
            closest_points_queue.push(Gridpoint{p_tmp, lengthSqr(p_tmp - source_point), GridpointDirection::y_pos});
            break;

        case GridpointDirection::y_neg:
            p_tmp.y -= 1;
            closest_points_queue.push(Gridpoint{p_tmp, lengthSqr(p_tmp - source_point), GridpointDirection::y_neg});
            break;

        case GridpointDirection::y_pos:
            p_tmp.y += 1;
            closest_points_queue.push(Gridpoint{p_tmp, lengthSqr(p_tmp - source_point), GridpointDirection::y_pos});
            break;
    }
}


std::tuple<cv::Point3f, tf2::Quaternion> getCenterAndRotation(const std::vector<cv::Point3f>& ordered_depth_points) {
    // Check if one point is off from al the others, if so: Exclude that point, else use the average.
    constexpr float thresh_factor = 2;  // Tweak
    constexpr float thresh_bias = 0.1;  // Tweak
    int invalid_point_idx{worstFitPointIndex(ordered_depth_points, thresh_factor, thresh_bias)};

    cv::Point3f center;
    tf2::Quaternion rotation;

    if (invalid_point_idx == -1) {
        center = getAverage(ordered_depth_points);
        // Only the following two ignored indicies give different results, can be shown by symmetry around center
        tf2::Quaternion rot_1 = getRotation(ordered_depth_points, 1, center);
        tf2::Quaternion rot_2 = getRotation(ordered_depth_points, 2, center);
        // Use the average of the rotations
        // TODO: Add test to check if the rotations are too dissimilar. Either return an error or do something smart with the covariance
        // See https://en.wikipedia.org/wiki/Slerp on how to average quarternions
        rotation = rot_1 + rot_2;
    }
    else {
        // Find the rotation given three points, and the center from the two opposite valid points
        switch (invalid_point_idx % 2) {
            case 0:
                center = (ordered_depth_points[1] + ordered_depth_points[3]) * 0.5;
                break;
            case 1:
                center = (ordered_depth_points[0] + ordered_depth_points[2]) * 0.5;
                break;
        }
        rotation = getRotation(ordered_depth_points, invalid_point_idx, center);
    }

    rotation.normalize();

    return std::tuple<cv::Point3f, tf2::Quaternion>(center, rotation);
}

tf2::Quaternion
getRotation(const std::vector<cv::Point3f> &ordered_points, const int ignore_point_idx, const cv::Point3f &center) {

    // There should be a total of three valid points given by "true" in valid_mask
    if (ordered_points.size() != 4) throw std::runtime_error("Corner points is not of size 4");

    // Points relative to the center
    std::vector<cv::Point3f> rel_pts{ordered_points - center};
    // Fill in the ignored point with the mirror of the opposite point, making the algorithm for the rotation matrix general
    rel_pts[ignore_point_idx] = - rel_pts[(ignore_point_idx+2) % 4];

    // Vectors for the rotation matrix:
    std::vector<cv::Point3f> rot_vecs(3);
    rot_vecs[0] = normalized(rel_pts[1] + rel_pts[2]);
    rot_vecs[1] = normalized(rel_pts[2] + rel_pts[3]);
    rot_vecs[2] = normalized(getCrossProd(rel_pts[1], rel_pts[2]));

    tf2::Matrix3x3 rot_matrix{rot_vecs[0].x, rot_vecs[1].x, rot_vecs[2].x,
                              rot_vecs[0].y, rot_vecs[1].y, rot_vecs[2].y,
                              rot_vecs[0].z, rot_vecs[1].z, rot_vecs[2].z};

    tf2::Quaternion ret;
    rot_matrix.getRotation(ret);

    // Thus far, the plate has been defined in the "optical frame" (x: right, y: down, z: forwards),
    // the following converts it to the "body frame" (x: forwards, y: left, z: up).
    // (See: https://www.ros.org/reps/rep-0103.html for standard orientations.)
    // Quaternion: (1 + i) * (1 + k) = 1 + i + k + ik = 1 + i - j + k
    const tf2::Quaternion frame_convertion{1, -1, 1, 1};
    ret *= frame_convertion;

    return ret;
}

int worstFitPointIndex(const std::vector<cv::Point3f> &points, float thresh_factor, float thresh_bias) {
    cv::Point3f center; double slope_x, slope_y;
    std::tie(center, slope_x, slope_y) = getPlaneFit(points);
    std::vector<float> plane_offset(4);
    for (int i{0}; i<4; ++i) {
        plane_offset[i] = std::abs(
                center.z +
                (points[i].x - center.x) * slope_x +
                (points[i].y - center.y) * slope_y
        );
    }
    float max_1_offset{0}, max_2_offset{0};
    int max_1_idx{-1};
    for (int i{0}; i<4; ++i) {
        if (plane_offset[i] > max_1_offset) {
            max_2_offset = max_1_offset;
            max_1_offset = plane_offset[i];
            max_1_idx = i;
        } else if(plane_offset[i] > max_2_offset) {
            max_2_offset = plane_offset[i];
        }
    }
    if (max_1_offset > max_2_offset * thresh_factor + thresh_bias) return max_1_idx;
    else return -1;
}

std::vector<cv::Point3f> getInnerCornerPoints(const cv::Mat &depth_image, const std::vector<cv::Point2f> &corner_points,
                                              const std::vector<double> &depth_camera_info_K_arr,
                                              const std::vector<double> &depth_camera_info_D, double scaling) {

    std::vector<cv::Point2f> inner_corner_points{getScaledTowardsCenter(corner_points, scaling)};
    std::vector<cv::Point3f> depth_corner_points = findXYZ(inner_corner_points, depth_image, depth_camera_info_K_arr);

    return depth_corner_points;

}

bool checkSquareness(const std::vector<cv::Point3f> &points, double scaling, int debug) {
    /// TODO: Figure out why the camera is scaling down by 1.6
    // Expected square size parameters in meters
    double side = 1.6 * 0.305 * (1-scaling); // 0.305 m == 1 foot == 12 inches
    double diag = side*1.414;

    double un_squareness = 0;
    un_squareness += pow(sqrt(lengthSqr(points.at(0) - points.at(1))) - side, 2);
    un_squareness += pow(sqrt(lengthSqr(points.at(1) - points.at(2))) - side, 2);
    un_squareness += pow(sqrt(lengthSqr(points.at(2) - points.at(3))) - side, 2);
    un_squareness += pow(sqrt(lengthSqr(points.at(3) - points.at(0))) - side, 2);
    un_squareness += pow(sqrt(lengthSqr(points.at(0) - points.at(2))) - diag, 2);
    un_squareness += pow(sqrt(lengthSqr(points.at(1) - points.at(3))) - diag, 2);

    un_squareness = sqrt(un_squareness);
    if (debug > 0)
        printf("un_sqr: %f\n", un_squareness);
    if (debug > 2) {

        std::vector<double> sqr_contrib{
                pow(sqrt(lengthSqr(points.at(0) - points.at(1))) - side, 2),
                pow(sqrt(lengthSqr(points.at(1) - points.at(2))) - side, 2),
                pow(sqrt(lengthSqr(points.at(2) - points.at(3))) - side, 2),
                pow(sqrt(lengthSqr(points.at(3) - points.at(0))) - side, 2),
                pow(sqrt(lengthSqr(points.at(0) - points.at(2))) - diag, 2),
                pow(sqrt(lengthSqr(points.at(1) - points.at(3))) - diag, 2),
        };
        std::vector<double> lengths {
            sqrt(lengthSqr(points.at(0) - points.at(1))),
            sqrt(lengthSqr(points.at(1) - points.at(2))),
            sqrt(lengthSqr(points.at(2) - points.at(3))),
            sqrt(lengthSqr(points.at(3) - points.at(0))),
            sqrt(lengthSqr(points.at(0) - points.at(2))),
            sqrt(lengthSqr(points.at(1) - points.at(3)))
        };

        printf("Sqr contribution: ");
        for (auto contribution : sqr_contrib)
            printf("%f, ", contribution);
        printf("\nLengths :");
        for (auto length : lengths)
            printf("%f, ", length);
        printf("\nSide, diag: %f, %f\n", side, diag);
    };

    return un_squareness < 0.5;
}

geometry_msgs::PoseWithCovarianceStamped
getCameraPoseWithCov(const std::vector<cv::Point3f> &depth_corner_points, int debug) {

    // Assign the center position and quaternion rotation from the four corner points to the poseWitCovStamped message
    cv::Point3f center; tf2::Quaternion rotation;
    std::tie(center, rotation) = getCenterAndRotation(depth_corner_points);

    geometry_msgs::PoseWithCovarianceStamped ret;
    // Set position and orientation
    ret.pose.pose.position.x = center.x;
    ret.pose.pose.position.y = center.y;
    ret.pose.pose.position.z = center.z;
    ret.pose.pose.orientation.w = rotation.getW();
    ret.pose.pose.orientation.x = rotation.getX();
    ret.pose.pose.orientation.y = rotation.getY();
    ret.pose.pose.orientation.z = rotation.getZ();

    // Set covariance
    for (int i{0}; i<6; ++i) {
        ret.pose.covariance.at(6 * i + i) = -1;
    }

    return ret;
}

