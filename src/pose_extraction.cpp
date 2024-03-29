//
// Created by marius on 07.11.2019.
//

#include "pose_extraction.h"

Pose_extraction::Pose_extraction(ros::NodeHandle &nh, image_transport::ImageTransport &it, int img_buffer_size)
            : nh_(nh), it_(it), BlueSquareScoreCalculator{}, image_ptr_buffer(img_buffer_size), debug{0}// TODO: Make buffer size parameter in launch file
{
    if (this->debug > 0) {
        // Initialize openCV window
        cv::namedWindow(INPUT_WINDOW);
        moveWindow(INPUT_WINDOW, 10, 10);
        cv::namedWindow(DEBUG_WINDOW);
        moveWindow(DEBUG_WINDOW, 10,540);
        cv::namedWindow(DEBUG_WINDOW2);
        moveWindow(DEBUG_WINDOW, 10,540);
    }
}

Pose_extraction::~Pose_extraction()
{
    if (this->debug > 0) {
        cv::destroyAllWindows();
    }
}

void Pose_extraction::bboxCb(const vision_msgs::Detection2D &bbox_msg) {
    if (debug % 2 == 1 && debug >= 3)
        printf("Got bbox  with seq %d: %d.%d nsec:  ", bbox_msg.header.seq, bbox_msg.header.stamp.sec, bbox_msg.header.stamp.nsec);

    cv::Rect bbox;
    if (!importBboxRect(bbox_msg, bbox)) {
        printf("Failed to import\n");
        return;
    }

    /// Get images saved by imageCb
    image_ptr_tuple images;
    if (!image_ptr_buffer.lookupById(bbox_msg.header.stamp, images)) {
        if (debug % 2 == 1)
            printf("Warning: Did not find image in lookupByID. Consider increasing buffer size.\n");
        return;
    };

    auto [cv_ptr_bgr, cv_ptr_depth] = images;

    auto [success, module_world_pose] = getWorldPose(cv_ptr_bgr->image, cv_ptr_depth->image, bbox);

    if (success) {
        this->pose_publisher.publish(module_world_pose);
    }

    if (this->debug > 0) {
        // Update GUI Windows
        if (!this->debug_window.empty() and success)
            cv::imshow(DEBUG_WINDOW, this->debug_window);
        cv::waitKey(1);
    };
    if (debug % 2 == 1 && debug >= 3) {
        if (success) printf("    Published pose\n");
        else printf("Failed to get world pose\n");
    }
}

void Pose_extraction::imageCb(const sensor_msgs::ImageConstPtr &bgr_msg, const sensor_msgs::ImageConstPtr &depth_msg) {
    if (debug % 2 == 1 && debug >= 3)
        printf("Got image with seq %d: %d.%d nsec:  ", bgr_msg->header.seq, bgr_msg->header.stamp.sec, bgr_msg->header.stamp.nsec);
    ros::Time t = bgr_msg->header.stamp;

    /// Image pointers
    cv_bridge::CvImagePtr cv_ptr_bgr;
    cv_bridge::CvImagePtr cv_ptr_depth;

    /// Try to import the images, if it fails: End this call by returning
    if (!importImageBgr(bgr_msg, cv_ptr_bgr)) {ROS_ERROR("Unable to import bgr-image."); return;}
    if (!importImageDepth(depth_msg, cv_ptr_depth)) {ROS_ERROR("Unable to import depth-image."); return;}
    std_msgs::Header header_in{bgr_msg->header};

    /// Save to lookupqueue buffer for retrieval in bboxCb
    image_ptr_tuple images{
            cv_ptr_bgr, cv_ptr_depth
            };

    image_ptr_buffer.push_elem(bgr_msg->header.stamp, images);
}

std::tuple<bool, geometry_msgs::PoseWithCovarianceStamped>
Pose_extraction::getWorldPose(const Mat &bgr_image, const Mat &depth_image, const Rect &bounding_box) {

    auto ret = geometry_msgs::PoseWithCovarianceStamped{};

    smoothed_depth_image = dilate_depth_image(depth_image, this->depth_dilation_kernel_size);
    depth_mask = generate_foreground_mask(smoothed_depth_image, this->depth_mean_offset_value, this->mask_dilation_kernel_size);

    /// Find the blue parts of the image:
    cv::Mat blueness_image;
    bluenessImageMasked(bgr_image, blueness_image, depth_mask, true);

    /// Find the corner points in the blueness image
    std::vector<cv::Point2f> corner_points;
    corner_points = findCornerPoints(bgr_image, blueness_image, bounding_box);

    if (!corner_points.empty() && this->has_depth_camera_info) {
        double scaling_towards_center{0.2};
        std::vector<cv::Point3f> inner_corner_points =
                getInnerCornerPoints(
                        depth_image, corner_points,
                        this->depth_camera_info_K_vec, this->depth_camera_info_D,
                        scaling_towards_center);

        if (checkSquareness(inner_corner_points, scaling_towards_center, this->debug)) {
            ret = getCameraPoseWithCov(inner_corner_points, this->debug);
            return std::tuple<bool, geometry_msgs::PoseWithCovarianceStamped>(true, ret);
        }
    }

    return std::tuple<bool, geometry_msgs::PoseWithCovarianceStamped>(false, ret);
}



bool Pose_extraction::importImageBgr(const sensor_msgs::ImageConstPtr &msg, cv_bridge::CvImagePtr &cv_ptr_out)
{
    /// Tries to import the image in the ROS message 'msg'. Returns true if success and false if failed.
    try
    {
        cv_ptr_out = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        return true;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }
};

bool Pose_extraction::importImageDepth(const sensor_msgs::ImageConstPtr &msg, cv_bridge::CvImagePtr &ptr_out)
{
    try
    {
        sensor_msgs::Image img;
        img.header = msg->header;
        img.height = msg->height;
        img.width = msg->width;
        img.is_bigendian = msg->is_bigendian;
        img.step = msg->step;
        img.data = msg->data;
        img.encoding = "mono16";
        ptr_out = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
        return true;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }
};

bool Pose_extraction::importBboxRect(const vision_msgs::Detection2D &bbox_msg, cv::Rect& rect_out) {
    double center_x{bbox_msg.bbox.center.x}, center_y{bbox_msg.bbox.center.y};
    double w{bbox_msg.bbox.size_x}, h{bbox_msg.bbox.size_y};

    rect_out.x = std::round(center_x - w/2);
    rect_out.y = std::round(center_y - h/2);
    rect_out.height = std::round(h);
    rect_out.width = std::round(w);

    return true;
};


std::vector<cv::Point2f> Pose_extraction::findCornerPoints(const Mat &cv_color_image, const cv::Mat &blueness_image,
                                                           const cv::Rect &inner_bounding_rect) {
    static constexpr double scaling = 0.2;  // Tweak
    int x{inner_bounding_rect.x}, y{inner_bounding_rect.y}, h{inner_bounding_rect.height}, w{inner_bounding_rect.width};
    cv::Rect outer_bounding_rect;
    outer_bounding_rect.x = x - scaling * w;
    outer_bounding_rect.y = y - scaling * h;
    outer_bounding_rect.width = w * (1 + 2*scaling);
    outer_bounding_rect.height = h * (1 + 2*scaling);

    return findCornerPoints(cv_color_image, blueness_image, inner_bounding_rect, outer_bounding_rect);
}


std::vector<cv::Point2f> Pose_extraction::findCornerPoints(const Mat &cv_color_image, const cv::Mat &blueness_image,
                                                           const cv::Rect &inner_bounding_rect,
                                                           cv::Rect outer_bounding_rect)
{
    // Make sure the bounding_rect contains the inner bounding rect and is not outside the window
    outer_bounding_rect = limitOuterCroppingRectangle(outer_bounding_rect, inner_bounding_rect, cv_color_image.cols, cv_color_image.rows);

    // Cropped image
    cv::Mat im_tmp = blueness_image(outer_bounding_rect);

    // blueness_image must be the grayscale-image with blue-subtraction to do canny on
    double canny_ratio = 2;                // Default: 2-3
    int const canny_max_lowThreshold = 80; // Default: 80
    int canny_kernel_size = 3;             // Default: 3
    // Canny edge image
    cv::Canny(im_tmp, im_tmp, canny_max_lowThreshold, canny_max_lowThreshold * canny_ratio);

    /// HoughLines
    enum class LINE_METHOD
    {
        Hough,
        HoughP
    };
    std::vector<cv::Vec2f> lines;
    switch (LINE_METHOD::HoughP)
    { // Tweak
    case LINE_METHOD::Hough:
        cv::HoughLines(im_tmp, lines, 1, CV_PI / 180, 30);
        break;

    case LINE_METHOD::HoughP:
        std::vector<cv::Vec4i> linepts;
        HoughLinesP(im_tmp, linepts, 1, CV_PI / 180, 30, 30, 10);
        // Convert to the format of hughLines of a line defined by r and theta
        linepointsToRadiusAngle(linepts, lines);

        break;
    }

    /// The good lines, thresholded lines
    std::vector<cv::Vec2f> lines_good;

    // Ignore the line if it is too far off the cartesian axes:
    constexpr int cutoff = 4; ///< The biggest accepted deviation from the +-x, +-y axes allowed for a line: pi/(2*cutoff)
    for (auto &line : lines)
    {
        double theta_m = std::fmod(line[1], CV_PI / 2);
        /// Why exactly does this give the correct cutoff? I thought it would cut twice as aggressively (half of the angle it currently does)
        if (CV_PI / (2 * cutoff) >= theta_m || theta_m >= (cutoff - 1) * CV_PI / (cutoff * 2))
            lines_good.emplace_back(line);
    };

    /// Draw the lines on the image
    // drawLines(cv_ptr_in->image, lines_good, 100, cv::Point{bounding_rect.x, bounding_rect.y});

    /// Find all the line intersections
    std::vector<cv::Point2f> intersections =
            computeMultiIntersections(lines_good, 3 * CV_PI / 8, cv::Point2f{0, 0});

    /// Remove all points outside the bounding rectangle. This is optional. Tweak (maybe expand the boundaries?
    intersections.erase(std::remove_if(intersections.begin(), intersections.end(),
                                       [outer_bounding_rect](cv::Point2f p) { return p.x < 0 || p.x > outer_bounding_rect.width ||
                                                                               p.y < 0 || p.y > outer_bounding_rect.height; }),
                        intersections.end());

    /// If there are no intersection points, there is nothing to work with
    if (intersections.empty())
        return std::vector<cv::Point2f>{};

    /// Do clustering of the points, and draw the cluster. Seems unnecessary when using convex hull,
    /// but might be relevant later. Is prone to errors anyway.
    /*
    {
        /// Do clustering of the points
        constexpr float epsilon_sqr = 100;
        constexpr int min_points = 2;
        DBSCAN clustering{min_points, epsilon_sqr, cvPointVecToDb(intersections)};
        clustering.run();
        std::vector<cv::Point2i> cluster_averages = clusterAveragesInt(clustering);

        /// Draw the different cluster points on the image
        drawClusteredPoints(cv_ptr_in->image, clustering, cv::Point2i{bounding_rect.x, bounding_rect.y});

        /// Draw the convex hull of the cluster averages in orange
        if (!cluster_averages.empty()) {
            /// Find convex hull of points
            // std::cout << "Clusters: " << cluster_averages.size() << "\n";
            std::vector<cv::Point2i> hull;
            cv::convexHull(cluster_averages, hull);

            drawCycle(cv_ptr_in->image, hull, cv::Point2i{bounding_rect.x, bounding_rect.y});
        }
    }
    */

    std::vector<cv::Point2i> hull;
    std::vector<cv::Point2i> tmp(intersections.size());
    for (int i{0}; i < intersections.size(); i++)
        tmp.at(i) = intersections.at(i);
    cv::convexHull(tmp, hull);

    std::vector<cv::Point2f> hull_approx;
    cv::approxPolyDP(hull, hull_approx, 1, true); // Tweak

    // If all points
    if (hull_approx.size() != 4)
    {
        if (this->debug > 0)
            std::printf("Did not find 4 points: %d \n", (int)hull_approx.size());
        return std::vector<cv::Point2f>{};
    }

    sortPointsClockwise(hull_approx);

    cv::Point2f offset{static_cast<float>(outer_bounding_rect.x), static_cast<float>(outer_bounding_rect.y)};
    double score = cornerPointScore(cv_color_image, hull_approx + offset);

    if (this->debug > 0)
    {
        // Draw bounding rectangles and convex hull on debug window
        cv_color_image.copyTo(this->debug_window);
        cv::rectangle(this->debug_window, inner_bounding_rect,
                      cv::Scalar{0, 256, 0});
        cv::rectangle(this->debug_window, outer_bounding_rect,
                      cv::Scalar{0, 0, 256});
        if (score > 0.2) {
            drawCycle(this->debug_window, hull_approx, cv::Point2i{outer_bounding_rect.x, outer_bounding_rect.y},
                      cv::Scalar{128, 0, 256}, true);
        }
        else {
            drawCycle(this->debug_window, hull_approx, cv::Point2i{outer_bounding_rect.x, outer_bounding_rect.y},
                      cv::Scalar{0, 128, 256}, true);
        }
    }
    if (score > 0.2)
        return hull_approx + offset;
    else
        return std::vector<cv::Point2f>{};
};

double Pose_extraction::cornerPointScore(const Mat &image_in, const std::vector<cv::Point2f> &corners_in)
{
    cv::Point2f dst_quad[4];

    for (int i = 0; i < 4; ++i)
    {
        dst_quad[i] = corners_in[i];
    }

    double res;
    res = BlueSquareScoreCalculator.getBlueSquareScore(image_in, dst_quad);

    if (this->debug > 0) {
        std::cout << "BSS result: " << res << "\n";
    }

    return res;
}

void Pose_extraction::depthCameraInfoCb(const boost::shared_ptr<sensor_msgs::CameraInfo const> &ptr_camera_info_message)
{
    // Return if camera info already has been retrieved.
    if (this->has_depth_camera_info) {
        return;
    }

    this->depth_camera_info_D = ptr_camera_info_message->D;

    const boost::array<double, 9> &K = ptr_camera_info_message->K;
    for (int i{0}; i < 9; ++i)
    {
        this->depth_camera_info_K_vec.at(i) = K.at(i);
    }

    this->has_depth_camera_info = true;
}

bool Pose_extraction::transformToWorld(geometry_msgs::PoseWithCovarianceStamped &pose_with_cov_stamped,
                                       const std_msgs::Header &from_header) {
    geometry_msgs::TransformStamped tfGeom;
    try {
        tfGeom = tf_buffer.lookupTransform(this->world_tf_frame_id, from_header.frame_id, from_header.stamp);
    }
    catch (tf2::TransformException &e) {
        if (this->debug >= 0) {
            printf("No transform found in pose_extraction with this error message:\n");
            printf("%s", e.what());
        }
        return false;
    }

    tf2::doTransform(pose_with_cov_stamped.pose.pose, pose_with_cov_stamped.pose.pose, tfGeom);

    // Fix metadata
    pose_with_cov_stamped.header.frame_id = this->world_tf_frame_id;
    pose_with_cov_stamped.header.stamp    = from_header.stamp;

    return true;
}


void bluenessImageMasked(const cv::Mat &im_in_bgr, cv::Mat &im_grey_out, const cv::Mat &depth_mask, const bool &blur)
{
    /// Calculate the image without the mask (and without blurring)
    bluenessImage(im_in_bgr, im_grey_out, false);

    /// Apply the mask:
    cv::bitwise_and(im_grey_out, depth_mask, im_grey_out);


    /// And blur afterwards
    if (blur)
        cv::GaussianBlur(im_grey_out, im_grey_out, cv::Size(17, 17), 3, 3);
}

void bluenessImage(const cv::Mat &im_in_bgr, cv::Mat &im_grey_out, const bool &blur)
{
    cv::Mat im_tmp;

    // Approx configs:
    //      GreyBlue official Lab values: {132, 125, 118}
    //      The following are samples with bright (sunlit) background in different color schemes:
    //          Direct Sunlight Outdoor:    {238, 108, 122}, HSV: { 90,  70, 249}, RGB: 181, 249, 249
    //          Sunlight outdoor:           {211, 104, 118}, HSV: {102, 115, 244}, RGB: 134, 200, 244
    //          Sun/shadow Outdoor:         {116, 129,  92}, HSV: {106, 167, 167}, RGB: 58,  110, 168
    //          Shadow/sun Outdoor:         { 84, 136,  92}, HSV: {109, 175, 137}, RGB: 43,  78,  137

    cv::cvtColor(im_in_bgr, im_grey_out, cv::COLOR_BGR2HSV);
    {
        cv::Scalar color{102, 110, 200};

        im_tmp.convertTo(im_tmp, CV_32SC3);
        im_grey_out.copyTo(im_tmp);

        cv::absdiff(im_tmp, color, im_tmp);

        /// Modify channels of HSV image seperately
        std::vector<cv::Mat> channels(3);
        cv::split(im_tmp, channels);
        // cv::multiply(channels[0], channels[0], channels[0]);
        // Hue:
        channels[0] = channels[0] * (3 * 255 / (180 * (0.114 * 3))); // Tweak
        // Saturation
        channels[1] = channels[1] * (0.0625 / (0.587 * 3)); //.025;  // Tweak
        // Value:
        channels[2] = channels[2] * (0.0625 / (0.299 * 3)); //.025;  // Tweak
        merge(channels, im_tmp);

        im_tmp = cv::abs(im_tmp);
        im_tmp.convertTo(im_grey_out, CV_8UC3);
        cv::cvtColor(im_grey_out, im_grey_out, cv::COLOR_BGR2GRAY);

        int trunc_part = 16; // Tweak  // For each pixel, multiply it by trunc_part and truncate the values over 255 to 255
        // The rest is truncated to max
        im_grey_out = im_grey_out * trunc_part;
    }


    cv::bitwise_not(im_grey_out, im_grey_out);

    if (blur)
        GaussianBlur(im_grey_out, im_grey_out, cv::Size(17, 17), 3, 3); // Tweak
}
