//
// Created by marius on 07.11.2019.
//

#include "pose_extraction.h"

Pose_extraction::Pose_extraction(ros::NodeHandle &nh, image_transport::ImageTransport &it) : nh_(nh), it_(it), BlueSquareScoreCalculator{}
{
    if (debug > 0) {
        // Initialize openCV window
        cv::namedWindow(INPUT_WINDOW);
        moveWindow(INPUT_WINDOW, 10, 10);
        // cv::namedWindow(RESULT_WINDOW);
        // moveWindow(RESULT_WINDOW, 720, 10);
        // cv::namedWindow(DEPTH_WINDOW);
        // moveWindow(DEPTH_WINDOW, 10,540);
        // cv::namedWindow(DEPTH_MASK_WINDOW);
        // moveWindow(DEPTH_MASK_WINDOW, 720,540);
    }
}

Pose_extraction::~Pose_extraction()
{
    if (debug > 0) {
        cv::destroyAllWindows();
    }
}

void Pose_extraction::imageCb(const sensor_msgs::ImageConstPtr &bgr_msg, const sensor_msgs::ImageConstPtr &depth_msg) {

    /// Try to import the images, if it fails: End this call by returning
    if (!importImageDepth(depth_msg, cv_ptr_depth)) {ROS_ERROR("Unable to import depth-image."); return;}
    if (!importImageBgr(bgr_msg, cv_ptr_bgr)) {ROS_ERROR("Unable to import bgr-image."); return;}
    std_msgs::Header header_in{bgr_msg->header};

    auto debug_timer_start = boost::chrono::high_resolution_clock::now();  // Timer

    /// Create depth mask thresholding away the background
    smoothed_depth_image = dilate_depth_image(cv_ptr_depth->image, this->depth_dilation_kernel_size);
    depth_mask = generate_foreground_mask(smoothed_depth_image, this->depth_mean_offset_value, this->mask_dilation_kernel_size);

    cv_ptr_bgr->image.copyTo(this->cv_work_image); // Deepcopy the image to the work image. This is so the work image is a three channel image.

    /// Find the blue parts of the image:
    cv::Mat blueness_image;
    greyFromImgConversion(cv_ptr_bgr->image, blueness_image, IMG_CONVERSION::HSV, depth_mask);
    blueness_image.copyTo(cv_work_image);

    /// Find a rectangle in which the blue square *at least* lies within
    cv::Rect bounding_rectangle;
    findOuterBoundingRectangle(cv_ptr_bgr, cv_work_image, bounding_rectangle);
    if (bounding_rectangle == cv::Rect{0, 0, 0, 0})
        return; // Return when no rectangle is found

    /// Do feature detection to find approximate position of image and confirm that the image contains
    /// the blue plate within the bounding rectangle
    std::vector<cv::Point2f> points_found;
    getInnerBoundingRectangle(bounding_rectangle, points_found);

    /// Find the exact transform in the given image
    std::vector<cv::Point2f> corner_points;
    bool is_blue_square;
    is_blue_square = findCornerPoints(cv_ptr_bgr, cv_work_image, points_found, bounding_rectangle, corner_points);

    // If succsessful, find pose and publish
    if (is_blue_square && this->has_depth_camera_info)
    {
        geometry_msgs::PoseWithCovarianceStamped pose_with_cov_stamped;  ///< Object for publishing
        getCameraPoseWithCov(cv_ptr_depth->image, corner_points,
                             this->depth_camera_info_K_arr, this->depth_camera_info_D,
                             pose_with_cov_stamped, debug);

        if (transform_to_world(header_in, pose_with_cov_stamped)) {
            this->pose_publisher.publish(pose_with_cov_stamped);
        };

    }

    if (debug > 0) {
        if (debug % 2) {
            // Timer endpoint and print
            auto stop = boost::chrono::high_resolution_clock::now();
            auto duration = boost::chrono::duration_cast<boost::chrono::milliseconds>(stop - debug_timer_start);
            std::string print_info = duration.count() < 10 ? "imageCb time: 0" : "imageCb time: ";
            print_info += to_string(duration.count()) + " ms";
            ROS_INFO("%s", print_info.c_str());
        }
        // Update GUI Windows
        cv::imshow(INPUT_WINDOW, cv_ptr_bgr->image);
        // cv::imshow(RESULT_WINDOW, blueness_image);
        // cv::imshow(DEPTH_WINDOW, smoothed_depth_image);
        // cv::imshow(DEPTH_MASK_WINDOW, depth_mask);
        cv::waitKey(1);
    }
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

void Pose_extraction::findOuterBoundingRectangle(const cv_bridge::CvImagePtr &cv_ptr_in, const cv::Mat &blueness_image,
                                                 cv::Rect &bounding_rectangle)
{

    cv::Mat cv_im_tmp_out; ///< Temporary output var, to be copied over cv_ptr_in->image before function return
    cv_ptr_in->image.copyTo(cv_im_tmp_out);

    /// Find a probable area in which the plate is

    // Blur:
    cv::Mat blueness_threshold_image;                                                              // TODO: Make work image
    cv::GaussianBlur(blueness_image, blueness_threshold_image, cv::Size(17, 17), 3, 3);            // Tweal
    cv::threshold(blueness_threshold_image, blueness_threshold_image, 80, 255, cv::THRESH_BINARY); // Tweak: Test adaptive threshold

    // Open then dilate the threshold
    cv::erode(blueness_threshold_image, blueness_threshold_image, cv::Mat(), cv::Point(-1, -1), 2);  // Tweak
    cv::dilate(blueness_threshold_image, blueness_threshold_image, cv::Mat(), cv::Point(-1, -1), 2); //Tweak

    // Find the largest areas
    cv::Mat labels, stats, centroids;
    cv::connectedComponentsWithStats(blueness_threshold_image, labels, stats, centroids);

    // Add the square-candidates to a list with a score
    std::vector<cv::Rect> candidates;
    std::vector<int> candidate_scores;
    cv::Scalar color(0, 255, 0);

    for (int i = 1; i < stats.rows; i++)
    {
        if (stats.at<int>(cv::Point(4, i)) < 1000)
            continue; // Tweak

        int x = stats.at<int>(cv::Point(0, i));
        int y = stats.at<int>(cv::Point(1, i));
        int w = stats.at<int>(cv::Point(2, i));
        int h = stats.at<int>(cv::Point(3, i));

        cv::Rect rect(x, y, w, h);
        cv::Rect featureDetectionRect(x - w / 2, y - w / 2, w * 2, h + w); // New verision as of 2020-03-03
        // cv::rectangle(blueness_threshold_image, rect, color);
        cv::rectangle(cv_im_tmp_out, rect, color);

        // The first int is the candidate score. The higher the better. Size * low position * squareness.
        candidates.emplace_back(featureDetectionRect);
        candidate_scores.emplace_back(stats.at<int>(cv::Point(4, i)) * (100 + y + h / 2) * ((8 * h) / w) * (16 - (8 * h) / w)); // TODO: This heuristic is ugly
    }
    // If we have no candidates, make a default one:
    if (candidates.empty())
    {
        bounding_rectangle = cv::Rect{0, 0, 0, 0};
    }
    else
    {
        int highestScoreIndex = std::max_element(candidate_scores.begin(), candidate_scores.end()) - candidate_scores.begin();
        bounding_rectangle = candidates.at(highestScoreIndex);
        cv::rectangle(cv_im_tmp_out, bounding_rectangle, cv::Scalar(0, 0, 255));
    }

    // TODO: Remove when making cv_ptr_in const
    cv_im_tmp_out.copyTo(cv_ptr_in->image);
}

bool Pose_extraction::findCornerPoints(cv_bridge::CvImagePtr &cv_ptr_in, cv::Mat &worked_image,
                                       const std::vector<cv::Point2f> &expected_corners, cv::Rect &bounding_rect,
                                       std::vector<cv::Point2f> &cornerpoints_out)
{
    cv::Mat im_out;
    cv_ptr_in->image.copyTo(im_out);
    cv::Mat im_tmp;
    cv::Mat im_tmp2;
    // Make sure the bounding_rect contains the expected corners
    for (const cv::Point2f &corner : expected_corners)
    {
        if (corner.x < bounding_rect.x)
            bounding_rect.x = corner.x;
        if (corner.x > bounding_rect.x + bounding_rect.width)
            bounding_rect.width = corner.x - bounding_rect.x;
        if (corner.y < bounding_rect.y)
            bounding_rect.y = corner.y;
        if (corner.y > bounding_rect.y + bounding_rect.width)
            bounding_rect.height = corner.y - bounding_rect.y;
    }
    // Make sure the bounding_rect is not outside the window
    limitCroppingRectangle(bounding_rect, cv_ptr_in->image.cols, cv_ptr_in->image.rows);

    cv::Mat worked_crop_image = worked_image(bounding_rect);

    // worked_image must be the grayscale-image with blue-subtraction to do canny on
    double canny_ratio = 2;                // Default: 2-3
    int const canny_max_lowThreshold = 80; // Default: 80
    int canny_kernel_size = 3;             // Default: 3
    cv::Canny(worked_crop_image, im_tmp, canny_max_lowThreshold, canny_max_lowThreshold * canny_ratio);
    im_tmp.copyTo(im_out);
    // TODO: This is for debug
    cv::Canny(worked_image, im_out, canny_max_lowThreshold, canny_max_lowThreshold * canny_ratio);

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
    //im_tmp.convertTo(cv_ptr_in->image, CV_U8C3);
    /// The good lines, thresholded lines
    std::vector<cv::Vec2f> lines_good;

    // TODO: Fix this line cutoff function
    // Ignore the line if it is too far off the cartesian axes:
    constexpr int cutoff = 4; ///< The biggest accepted deviation from the +-x, +-y axes allowed for a line: pi/(2*cutoff)
    for (auto &line : lines)
    {

        double theta_m = std::fmod(line[1], CV_PI / 2);

        /// Why exactly does this give the correct cutoff? I thought it would cut twice as aggressively (half of the angle it currently does)
        if (CV_PI / (2 * cutoff) >= theta_m || theta_m >= (cutoff - 1) * CV_PI / (cutoff * 2))
        {
            lines_good.emplace_back(line);
        };
    };

    /// Draw the lines on the image
    // drawLines(cv_ptr_in->image, lines_good, 100, cv::Point{bounding_rect.x, bounding_rect.y});

    /// Find all the line intersections
    std::vector<cv::Point2f> intersections = computeMultiIntersections(lines_good, 3 * CV_PI / 8, cv::Point2f{0, 0});

    /// Remove all points outside the bounding rectangle. This is optional. Tweak (maybe expand the boundaries?
    intersections.erase(std::remove_if(intersections.begin(), intersections.end(),
                                       [bounding_rect](cv::Point2f p) { return p.x < 0 || p.x > bounding_rect.width ||
                                                                               p.y < 0 || p.y > bounding_rect.height; }),
                        intersections.end());

    /// If there are no intersection points, there is nothing to work with
    if (intersections.empty())
        return false;

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

    /// Draw the convex hull of all the points in purple
    std::vector<cv::Point2i> hull;
    if (!intersections.empty())
    {
        std::vector<cv::Point2i> intersections_int(intersections.size());
        for (unsigned long i{0}; i < intersections.size(); i++)
        {
            intersections_int.at(i) = intersections.at(i);
        }

        cv::convexHull(intersections_int, hull);

        //        drawCycle(cv_ptr_in->image, hull, cv::Point2i{bounding_rect.x, bounding_rect.y},
        //                  cv::Scalar{128, 0, 256});
    }
    //void onLinePointsRemoved(hull, line_dist_thresh)

    std::vector<cv::Point2f> hull_approx;
    cv::approxPolyDP(hull, hull_approx, 1, true); // Tweak

    double score;
    if (hull_approx.size() == 4)
    {
        sortPointsClockwise(hull_approx);
        /// Draw the points
        /*cv::circle(cv_ptr_in->image, cv::Point2f{hull_approx[0].x + bounding_rect.x, hull_approx[0].y + bounding_rect.y},
                   1, cv::Scalar{255, 0, 0}, 2);
        cv::circle(cv_ptr_in->image, cv::Point2f{hull_approx[1].x + bounding_rect.x, hull_approx[1].y + bounding_rect.y},
                   1, cv::Scalar{0, 255, 0}, 2);
        cv::circle(cv_ptr_in->image, cv::Point2f{hull_approx[2].x + bounding_rect.x, hull_approx[2].y + bounding_rect.y},
                   1, cv::Scalar{0, 0, 255}, 2);
        cv::circle(cv_ptr_in->image, cv::Point2f{hull_approx[3].x + bounding_rect.x, hull_approx[3].y + bounding_rect.y},
                   1, cv::Scalar{0, 0, 0}, 2);*/
        /// ---------------------------------------------------------------
        /// Torleiv the man
        /// ---------------------------------------------------------------
        cv::Point2f offset{static_cast<float>(bounding_rect.x), static_cast<float>(bounding_rect.y)};
        score = cornerPointScore(cv_ptr_in->image, worked_image,
                                 hull_approx + offset);

        if (score > 0.2)
        {
            drawCycle(cv_ptr_in->image, hull_approx, cv::Point2i{bounding_rect.x, bounding_rect.y},
                      cv::Scalar{128, 0, 256}, true);

            cornerpoints_out = hull_approx + offset;
            im_out.copyTo(debug_window);
        }
        else
        {
            drawCycle(cv_ptr_in->image, hull_approx, cv::Point2i{bounding_rect.x, bounding_rect.y},
                      cv::Scalar{0, 128, 256}, true);
        }
    }

    im_out.copyTo(debug_window);

    if (hull_approx.size() > 4)
    {
        if (debug >= 0)
            std::printf("Found more than 4 points: %d \n", (int)hull_approx.size());
        return false;
    }
    else if (hull_approx.size() > 4)
    {
        if (debug >= 0)
            std::printf("Found less than 4 points: %d \n", (int)hull_approx.size());
        return false;
    }
    else
    {
        return score > 0.2;
    }

    /// Find probable good corners with goodFeaturesToTrack. Not promising as of 2020-02-27,
    /// but might improve with better blueness image
    /*
    std::vector<cv::Point2f> gftt_corners;
    constexpr int gfft_max_corners{50};
    constexpr float gftt_qualitylevel{0.01};
    constexpr float gftt_min_distance{5};
    cv::goodFeaturesToTrack(worked_crop_image, gftt_corners,
            gfft_max_corners, gftt_qualitylevel, gftt_min_distance);

    drawPoints(cv_ptr_in->image, gftt_corners, cv::Point2i{bounding_rect.x, bounding_rect.y});
*/
};

void Pose_extraction::getInnerBoundingRectangle(const cv::Rect &bounding_rect, std::vector<cv::Point2f> &points_out)
{

    // Simply put the bounding box initial variables in the points_found
    // Do the inverse of the bounding-rectangle transform in houghLinesIntersection
    int x = bounding_rect.x;
    int y = bounding_rect.y;
    int w = bounding_rect.width;
    int h = bounding_rect.height;

    w = w / 2;
    h = h - w;
    x = x + w / 2;
    y = y + w / 2; // New version as of 2020-03-03
    points_out.emplace_back(cv::Vec2i(x, y));
    points_out.emplace_back(cv::Vec2i(x + w, y));
    points_out.emplace_back(cv::Vec2i(x + w, y + h));
    points_out.emplace_back(cv::Vec2i(x, y + h));
}

double Pose_extraction::cornerPointScore(cv::Mat &image_in, cv::Mat &blueness_image, const std::vector<cv::Point2f> &corners_in)
{
    /*Testing only*/ cv::Point2f dst_quad[4];
    /* This is for testing only
    dst_quad[0] =  cv::Point2f(265, 120);
    dst_quad[1] =  cv::Point2f((float)image_in.cols-290, 120);
    dst_quad[2] =  cv::Point2f((float)image_in.cols-290, (float)image_in.rows-270);
    dst_quad[3] =  cv::Point2f(265, (float)image_in.rows-270);
    // */

    for (int i = 0; i < 4; ++i)
    {
        dst_quad[i] = corners_in[i];
    }

    auto debug_start_bss = boost::chrono::high_resolution_clock::now();

    double res;
    res = BlueSquareScoreCalculator.getBlueSquareScore(image_in, dst_quad);

    if (debug > 0) {
        std::cout << "BSS result: " << res << "\n";
        if (debug % 2) {
            auto stop_bss = boost::chrono::high_resolution_clock::now();
            auto duration_bss = boost::chrono::duration_cast<boost::chrono::milliseconds>(stop_bss - debug_start_bss);
            std::string print_info = duration_bss.count() < 10 ? "BSS_Function time: 0" : "BSS_Function time: ";
            print_info += to_string(duration_bss.count()) + " ms";
            ROS_INFO("%s", print_info.c_str());
        }
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
        this->depth_camera_info_K_arr.at(i) = K.at(i);
    }

    this->has_depth_camera_info = true;
}

bool Pose_extraction::transform_to_world(const std_msgs::Header &from_header, geometry_msgs::PoseWithCovarianceStamped &pose_with_cov_stamped) {

    geometry_msgs::TransformStamped tfGeom;
    try {
        tfGeom = tf_buffer.lookupTransform(this->world_tf_frame_id, from_header.frame_id, from_header.stamp);
    }
    catch (tf2::TransformException &e) {
        if (debug >= 0) {
            printf("No transform found in pose_extraction with this error messake:\n");
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

bool acceptLinePair(const cv::Vec2f &line1, const cv::Vec2f &line2, const float &minTheta)
{
    float theta1 = line1[1], theta2 = line2[1];

    if (theta1 >= CV_PI)
        theta1 -= CV_PI;
    if (theta2 >= CV_PI)
        theta2 -= CV_PI;

    if (std::min(abs(theta1 - theta2), abs(theta2 - theta1)) < minTheta)
        return false;

    if (theta1 < theta2)
    {
        return (theta1 + CV_PI - theta2) > minTheta;
    }
    else
    {
        return (theta2 + CV_PI - theta1) > minTheta;
    };
};

cv::Point2f computeIntersect(const cv::Vec2f &line1, const cv::Vec2f &line2, const cv::Point2f &offset)
{
    std::vector<cv::Point2f> p1 = lineToPointPair(line1);
    std::vector<cv::Point2f> p2 = lineToPointPair(line2);

    float denom = (p1[0].x - p1[1].x) * (p2[0].y - p2[1].y) - (p1[0].y - p1[1].y) * (p2[0].x - p2[1].x);
    cv::Point2f intersect(((p1[0].x * p1[1].y - p1[0].y * p1[1].x) * (p2[0].x - p2[1].x) -
                           (p1[0].x - p1[1].x) * (p2[0].x * p2[1].y - p2[0].y * p2[1].x)) /
                                  denom +
                              offset.x,
                          ((p1[0].x * p1[1].y - p1[0].y * p1[1].x) * (p2[0].y - p2[1].y) -
                           (p1[0].y - p1[1].y) * (p2[0].x * p2[1].y - p2[0].y * p2[1].x)) /
                                  denom +
                              offset.y);

    return intersect;
}

bool limitCroppingRectangle(cv::Rect &rect, const int &im_width, const int &im_height)
{
    bool changed = false;
    if (rect.x < 0)
    {
        rect.width += rect.x;
        rect.x = 0;
        changed = true;
    }
    if (rect.x + rect.width > im_width)
    {
        rect.width = im_width - rect.x;
        changed = true;
    }
    if (rect.y < 0)
    {
        rect.height += rect.y;
        rect.y = 0;
        changed = true;
    }
    if (rect.y + rect.height > im_height)
    {
        rect.height = im_height - rect.y;
        changed = true;
    }
    return changed;
}

std::vector<cv::Point2f> lineToPointPair(const cv::Vec2f &line)
{
    std::vector<cv::Point2f> points{2};

    float r = line[0], t = line[1];
    double cos_t = cos(t), sin_t = sin(t);
    double x0 = r * cos_t, y0 = r * sin_t;
    double alpha = 1000;

    points[0] = cv::Point2f(x0 + alpha * (-sin_t), y0 + alpha * cos_t);
    points[1] = cv::Point2f(x0 - alpha * (-sin_t), y0 - alpha * cos_t);

    return points;
}

void greyFromImgConversion(const cv::Mat &im_in_bgr, cv::Mat &im_grey_out, IMG_CONVERSION conversion_type, const cv::Mat &depth_mask, const bool &blur)
{
    /// Calculate the image without the mask (and without blurring)
    greyFromImgConversion(im_in_bgr, im_grey_out, conversion_type, false);

    /// Apply the mask:
    cv::bitwise_and(im_grey_out, depth_mask, im_grey_out);

    /// And blur afterwards
    if (blur)
        GaussianBlur(im_grey_out, im_grey_out, cv::Size(17, 17), 3, 3);
}

void greyFromImgConversion(const cv::Mat &im_in_bgr, cv::Mat &im_grey_out, IMG_CONVERSION conversion_type, const bool &blur)
{
    cv::Mat im_tmp;

    // Approx configs:
    //      GreyBlue official Lab values: {132, 125, 118}
    //      The following are samples with bright (sunlit) background in different color schemes:
    //          Direct Sunlight Outdoor:    {238, 108, 122}, HSV: { 90,  70, 249}, RGB: 181, 249, 249
    //          Sunlight outdoor:           {211, 104, 118}, HSV: {102, 115, 244}, RGB: 134, 200, 244
    //          Sun/shadow Outdoor:         {116, 129,  92}, HSV: {106, 167, 167}, RGB: 58,  110, 168
    //          Shadow/sun Outdoor:         { 84, 136,  92}, HSV: {109, 175, 137}, RGB: 43,  78,  137

    switch (conversion_type)
    {
    case IMG_CONVERSION::Lab:
    {
        cv::cvtColor(im_in_bgr, im_grey_out, cv::COLOR_BGR2Lab);
        cv::Scalar color{132, 125, 118};

        im_tmp.convertTo(im_tmp, CV_32SC3);
        im_grey_out.copyTo(im_tmp);
        cv::absdiff(im_tmp, color, im_tmp);

        //std::string test_string{"Color of middle pixel: J: " +
        //                        std::to_string(im_tmp.at<cv::Vec3b>(400, 600)[0]) + "  a: " +
        //                        std::to_string(im_tmp.at<cv::Vec3b>(400, 600)[1]) + "  b: " +
        //                        std::to_string(im_tmp.at<cv::Vec3b>(400, 600)[2]) + "\n"};
        //std::cout << test_string;

        // Modify channels of Lab image seperately

        std::vector<cv::Mat> channels(3);
        cv::split(im_tmp, channels);
        channels[0] = channels[0] * 0.2 / (0.114 * 3); //* 0.25;  // Tweak
        //cv::multiply(channels[0] - 0, channels[0] * 0.04, channels[0]);  // Tweak
        channels[1] = channels[1] * 1 / (0.587 * 3); // Tweak
        channels[2] = channels[2] * 1 / (0.299 * 3); // Tweak
        merge(channels, im_tmp);

        im_tmp = cv::abs(im_tmp);
        im_tmp.convertTo(im_grey_out, CV_8UC3);
        cv::cvtColor(im_grey_out, im_grey_out, cv::COLOR_BGR2GRAY);

        int trunc_part = 32; // Tweak  // For each pixel, multiply it by trunc_part and truncate the values over 255 to 255
        // The rest is truncated to max
        im_grey_out = im_grey_out * trunc_part;
    }
    break;

    case IMG_CONVERSION::HSV:
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
        break;

    case IMG_CONVERSION::Lab_MultiThresh:
        cv::cvtColor(im_in_bgr, im_grey_out, cv::COLOR_BGR2HSV);
        im_tmp.convertTo(im_tmp, CV_32SC3);
        im_grey_out.copyTo(im_tmp);
        {

            // Thresholding H-values:
            constexpr int THRESH_L_LOWER{70}, THRESH_L_UPPER{250};
            constexpr int THRESH_a_LOWER{90}, THRESH_a_UPPER{145};
            constexpr int THRESH_b_LOWER{60}, THRESH_b_UPPER{130};

            // Debug
            //std::cout << "Im_tmp: " << matType2str(im_tmp.type())
            //          << "    im_grey_out: " << matType2str(im_grey_out.type()) << "\n";

            // Debug
            {
                //std::string test_string{"Color of middle pixel: H: " +
                ///                        std::to_string(im_tmp.at<cv::Vec3b>(400, 600)[0]) + "  S: " +
                ///                        std::to_string(im_tmp.at<cv::Vec3b>(400, 600)[1]) + "  V: " +
                ///                        std::to_string(im_tmp.at<cv::Vec3b>(400, 600)[2]) + "\n"};
                ///std::cout << test_string;
            }

            // Modify channels of Lab image seperately

            {
                std::vector<cv::Mat> channels(3);
                cv::split(im_tmp, channels);

                cv::threshold(channels[0], channels[0], THRESH_L_UPPER, 255, cv::THRESH_TOZERO_INV);
                cv::threshold(channels[0], channels[0], THRESH_L_LOWER, 255, cv::THRESH_BINARY);
                cv::threshold(channels[1], channels[1], THRESH_a_UPPER, 255, cv::THRESH_TOZERO_INV);
                cv::threshold(channels[1], channels[1], THRESH_a_LOWER, 255, cv::THRESH_BINARY);
                cv::threshold(channels[2], channels[2], THRESH_b_UPPER, 255, cv::THRESH_TOZERO_INV);
                cv::threshold(channels[2], channels[2], THRESH_b_LOWER, 255, cv::THRESH_BINARY);

                channels[0] = channels[0] & channels[1] & channels[2];
                channels[0].copyTo(im_tmp);
                //merge(channels, im_tmp);
            }
        }
        im_tmp.convertTo(im_grey_out, CV_8UC3);
        break;

    case IMG_CONVERSION::HSV_MultiThresh:
        cv::cvtColor(im_in_bgr, im_grey_out, cv::COLOR_BGR2Lab);
        im_tmp.convertTo(im_tmp, CV_32SC3);
        im_grey_out.copyTo(im_tmp);

        // Thresholding H-values:
        {
            constexpr int THRESH_H_LOWER{95}, THRESH_H_UPPER{125};
            constexpr int THRESH_S_LOWER{60}, THRESH_S_UPPER{190};
            constexpr int THRESH_V_LOWER{100}, THRESH_V_UPPER{255};

            // Debug
            {
                //std::string test_string{"Color of middle pixel: H: " +
                ///                        std::to_string(im_tmp.at<cv::Vec3b>(400, 600)[0]) + "  S: " +
                ///                        std::to_string(im_tmp.at<cv::Vec3b>(400, 600)[1]) + "  V: " +
                ///                        std::to_string(im_tmp.at<cv::Vec3b>(400, 600)[2]) + "\n"};
                ///std::cout << test_string;
            }

            // Modify channels of Lab image seperately

            {
                std::vector<cv::Mat> channels(3);
                cv::split(im_tmp, channels);

                cv::threshold(channels[0], channels[0], THRESH_H_UPPER, 255, cv::THRESH_TOZERO_INV);
                cv::threshold(channels[0], channels[0], THRESH_H_LOWER, 255, cv::THRESH_BINARY);
                cv::threshold(channels[1], channels[1], THRESH_S_UPPER, 255, cv::THRESH_TOZERO_INV);
                cv::threshold(channels[1], channels[1], THRESH_S_LOWER, 255, cv::THRESH_BINARY);
                cv::threshold(channels[2], channels[2], THRESH_V_UPPER, 255, cv::THRESH_TOZERO_INV);
                cv::threshold(channels[2], channels[2], THRESH_V_LOWER, 255, cv::THRESH_BINARY);

                channels[0] = channels[0] & channels[1] & channels[2];
                channels[0].copyTo(im_tmp);
                //merge(channels, im_tmp);
            }
        }
        im_tmp.convertTo(im_grey_out, CV_8UC3);
        break;
    }

    cv::bitwise_not(im_grey_out, im_grey_out);

    if (blur)
        GaussianBlur(im_grey_out, im_grey_out, cv::Size(17, 17), 3, 3); // Tweak
}

void linepointsToRadiusAngle(const std::vector<cv::Vec4i> &lines_points_in, std::vector<cv::Vec2f> &lines_out)
{
    cv::Vec2f line;
    for (auto &pts : lines_points_in)
    {
        // pts is on the form [pt1.x, pt1.y, pt2.x, pt2.y]

        double theta, r, a, b; // Assuming the line is on the form y=ax+b
        if (pts[0] - pts[2] == 0)
        {
            // Handling the case of a vertical line
            theta = 0;
            r = pts[0];
        }
        else if (pts[1] - pts[3] == 0)
        {
            theta = CV_PI / 2;
            r = pts[1];
        }
        else
        {
            a = (pts[3] - pts[1]) / static_cast<double>(pts[2] - pts[0]); // a = (y_1 - y_0) / (x_1 - x_0)
            b = pts[3] - a * pts[2];                                      // b = y_1 - a*x_1
            theta = std::atan(-1 / a);
            r = std::sin(theta) * b;
        }

        line[0] = r;
        line[1] = theta;
        lines_out.emplace_back(line);
    }
}

std::vector<cv::Point2f>
computeMultiIntersections(const std::vector<cv::Vec2f> &lines, const float &minAngleDiff,
                          const cv::Point2f &offset, const int &max_intersects)
{
    std::vector<cv::Point2f> intersections{};
    int remaining_intersects{max_intersects};

    // Loop thorugh each line pair once
    for (unsigned int i = 0; i < lines.size(); i++)
    {
        for (unsigned int j = 0; j < i; j++)
        {
            if (acceptLinePair(lines[i], lines[j], minAngleDiff)) // Reject the lines if they are too parallel
            {
                intersections.push_back(computeIntersect(lines[i], lines[j], offset));

                if (--remaining_intersects == 0)
                    return intersections;
            }
        }
    }

    return intersections;
}

std::string matType2str(int type)
{
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth)
    {
    case CV_8U:
        r = "8U";
        break;
    case CV_8S:
        r = "8S";
        break;
    case CV_16U:
        r = "16U";
        break;
    case CV_16S:
        r = "16S";
        break;
    case CV_32S:
        r = "32S";
        break;
    case CV_32F:
        r = "32F";
        break;
    case CV_64F:
        r = "64F";
        break;
    default:
        r = "User";
        break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}

template <typename T>
std::vector<Pointdb> cvPointVecToDb(const std::vector<T> &points)
{
    std::vector<Pointdb> ret;
    for (const auto &point : points)
    {
        ret.emplace_back(Pointdb{static_cast<float>(point.x), static_cast<float>(point.y), 0, -1});
    };
    return ret;
};

std::vector<cv::Point2f> dbPointVecToCv(const std::vector<Pointdb> &points)
{
    std::vector<Point2f> ret;

    for (const auto &point : points)
    {
        ret.emplace_back(cv::Point2f{point.x, point.y});
    };
    return ret;
};

std::vector<cv::Point2i> clusterAveragesInt(const DBSCAN &clustering, bool with_noise)
{
    const std::vector<Pointdb> &clustered_points = clustering.getPoints();

    /// initialize averages with vectors of (0, 0, 0) for (x_sum, y_sum, total_points)
    std::vector<cv::Vec3f> cluster_sum(clustering.getMaxClusterID());
    std::vector<cv::Point2i> ret_points;

    for (const auto &point : clustered_points)
    {
        if (point.clusterID == -1)
        {
            if (with_noise)
                ret_points.emplace_back(cv::Point2i{static_cast<int>(std::lround(point.x)), static_cast<int>(std::lround(point.y))});
        }
        else
        {
            cluster_sum.at(point.clusterID - 1)[0] += point.x;
            cluster_sum.at(point.clusterID - 1)[1] += point.y;
            cluster_sum.at(point.clusterID - 1)[2] += 1;
        }
    }

    for (const auto &sum : cluster_sum)
    {
        if (sum[2] > 0)
        {
            ret_points.emplace_back(cv::Point2i{static_cast<int>(sum[0] / sum[2]), static_cast<int>(sum[1] / sum[2])});
        }
    }
    return ret_points;
}

void printPwC(const geometry_msgs::PoseWithCovarianceStamped& p) {
    printf("Pose: %f, %f, %f, \nOrient: %f, %f, %f, %f, ",
           p.pose.pose.position.x, p.pose.pose.position.y, p.pose.pose.position.z,
           p.pose.pose.orientation.x, p.pose.pose.orientation.y,
           p.pose.pose.orientation.z, p.pose.pose.orientation.w);
}



/*void Pose_extraction::test() {
    ROS_INFO("Pose_extraction::test run");
    {
        tf_pub.header.stamp = ros::Time::now();
        tf_pub.header.frame_id = "Pose_extraction_test";
        tf_pub.child_frame_id = "none_test";
        tf_pub.transform.translation.x = 0.0;
        tf_pub.transform.translation.y = 0.0;
        tf_pub.transform.translation.z = 0.0;
        tf_pub.transform.rotation.x = 0;
        tf_pub.transform.rotation.y = 0;
        tf_pub.transform.rotation.z = 0;
        tf_pub.transform.rotation.w = 0;
        tf_broadcaster.sendTransform(tf_pub);
    }
}*/
