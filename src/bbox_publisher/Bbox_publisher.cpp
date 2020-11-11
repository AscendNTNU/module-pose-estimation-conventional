//
// Created by marius on 30.10.2020.
//

#include "Bbox_publisher.h"

Bbox_publisher::Bbox_publisher(ros::NodeHandle &nh, image_transport::ImageTransport &it) : nh_(nh), it_(it), debug{0} {}

void Bbox_publisher::imageCb(const sensor_msgs::ImageConstPtr &bgr_msg, const sensor_msgs::ImageConstPtr &depth_msg) {
    /// Image pointers
    cv_bridge::CvImagePtr cv_ptr_bgr;
    cv_bridge::CvImagePtr cv_ptr_depth;

    /// Try to import the images, if it fails: End this call by returning
    if (!importImageDepth(depth_msg, cv_ptr_depth)) {ROS_ERROR("Unable to import depth-image."); return;}
    if (!importImageBgr(bgr_msg, cv_ptr_bgr)) {ROS_ERROR("Unable to import bgr-image."); return;}
    std_msgs::Header header_in{bgr_msg->header};

    auto debug_timer_start = boost::chrono::high_resolution_clock::now();  // Timer

    /// Create depth mask thresholding away the background
    smoothed_depth_image = dilate_depth_image(cv_ptr_depth->image, this->depth_dilation_kernel_size);
    depth_mask = generate_foreground_mask(smoothed_depth_image, this->depth_mean_offset_value, this->mask_dilation_kernel_size);

    /// Find the blue parts of the image:
    cv::Mat blueness_image;
    bluenessImageMasked(cv_ptr_bgr->image, blueness_image, depth_mask, true);

    /// Find a rectangle in which the blue square *at least* lies within
    auto [inner_bounding_rectangle, outer_bounding_rectangle] =
            getBoundingRectangle(blueness_image);
    if (inner_bounding_rectangle == cv::Rect{0, 0, 0, 0}) {
        return; // Return when no rectangle is found
    }

    publishDetection(inner_bounding_rectangle, header_in);

    if (this->debug > 0) {
        if (this->debug % 2) {
            // Timer endpoint and print
            auto stop = boost::chrono::high_resolution_clock::now();
            auto duration = boost::chrono::duration_cast<boost::chrono::milliseconds>(stop - debug_timer_start);
            std::string print_info = duration.count() < 10 ? "bbox_publisher_cb time: 0" : "bbox_publisher_cb time: ";
            print_info += std::to_string(duration.count()) + " ms";
            ROS_INFO("%s", print_info.c_str());
        }
    }
}

void Bbox_publisher::publishDetection(const cv::Rect &bounding_rect, const std_msgs::Header &header) {
    vision_msgs::Detection2D msg;

    // Set header
    msg.header = header;

    // Set bounding box
    double x{static_cast<double>(bounding_rect.x)}, y{static_cast<double>(bounding_rect.y)};
    double w{static_cast<double>(bounding_rect.width)}, h{static_cast<double>(bounding_rect.height)};
    msg.bbox.center.x = x + w/2;
    msg.bbox.center.y = y + h/2;
    msg.bbox.center.theta = 0;
    msg.bbox.size_x = w;
    msg.bbox.size_y = h;

    // Set detection score
    msg.results = std::vector<vision_msgs::ObjectHypothesisWithPose>(1);
    msg.results.at(0).id = 0;
    msg.results.at(0).score = 1;
    // msg.results.at(0).pose = geometry_msgs::PoseWithCovariance{};

    // Set source image
    // msg.source_img = sensor_msgs::Image{};  // TODO: Might be used for blue image publication for efficiency

    bbox_publisher.publish(msg);
}

bool importImageBgr(const sensor_msgs::ImageConstPtr &msg, cv_bridge::CvImagePtr &cv_ptr_out)
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

bool importImageDepth(const sensor_msgs::ImageConstPtr &msg, cv_bridge::CvImagePtr &ptr_out)
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

std::tuple<cv::Rect, cv::Rect> getBoundingRectangle(const cv::Mat &blueness_image)
{
    /// Find a probable area in which the plate is

    // Blur:
    cv::Mat blueness_threshold_image;                                                              // TODO: Make work image
    cv::GaussianBlur(blueness_image, blueness_threshold_image, cv::Size(17, 17), 3, 3);            // Tweal

    // Threshold vals
    cv::threshold(blueness_threshold_image, blueness_threshold_image, 80, 255, cv::THRESH_BINARY); // Tweak: Test adaptive threshold

    // Erode then dilate the threshold
    cv::erode(blueness_threshold_image, blueness_threshold_image, cv::Mat(), cv::Point(-1, -1), 2);  // Tweak
    cv::dilate(blueness_threshold_image, blueness_threshold_image, cv::Mat(), cv::Point(-1, -1), 2); //Tweak

    // Find the largest areas
    cv::Mat labels, stats, centroids;
    cv::connectedComponentsWithStats(blueness_threshold_image, labels, stats, centroids);

    // Add the square-candidates to a list with a score
    std::vector<cv::Rect> candidates;
    std::vector<int> candidate_scores;

    for (int i = 1; i < stats.rows; i++)
    {
        if (stats.at<int>(cv::Point(4, i)) < 1000)
            continue; // Tweak

        int x = stats.at<int>(cv::Point(0, i));
        int y = stats.at<int>(cv::Point(1, i));
        int w = stats.at<int>(cv::Point(2, i));
        int h = stats.at<int>(cv::Point(3, i));

        cv::Rect rect(x, y, w, h);
        // cv::rectangle(blueness_threshold_image, rect, color);

        // The first int is the candidate score. The higher the better. Size * low position * squareness.
        candidates.emplace_back(rect);
        candidate_scores.emplace_back(stats.at<int>(cv::Point(4, i)) * (100 + y + h / 2) * ((8 * h) / w) * (16 - (8 * h) / w)); // TODO: This heuristic is ugly
    }
    // If we have no candidates, return a dummy value:
    if (candidates.empty())
    {
        return {cv::Rect{0, 0, 0, 0}, cv::Rect{0, 0, 0, 0}};
    }

    // Find highest scoring candidate
    int highestScoreIndex = std::max_element(candidate_scores.begin(), candidate_scores.end()) - candidate_scores.begin();
    cv::Rect bounding_rect = candidates.at(highestScoreIndex);

    // Create an outer bounding rectangle with additional padding
    cv::Rect outer_bounding_rect(
            bounding_rect.x - bounding_rect.width / 2,
            bounding_rect.y - bounding_rect.width / 2,
            bounding_rect.width * 2,
            bounding_rect.height + bounding_rect.width
    );

    return {bounding_rect, outer_bounding_rect};

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
