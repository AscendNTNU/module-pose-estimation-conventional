//
// Created by marius on 07.11.2019.
//

#include "pose_extraction.h"

Pose_extraction::Pose_extraction(ros::NodeHandle &nh, image_transport::ImageTransport &it, int img_buffer_size)
            : nh_(nh), it_(it), image_ptr_buffer(img_buffer_size), debug{0}
{
    if (this->debug > 0) {
        // Initialize openCV window
        cv::namedWindow(INPUT_WINDOW);
        cv::moveWindow(INPUT_WINDOW, 10, 10);
        cv::namedWindow(DEBUG_WINDOW);
        cv::moveWindow(DEBUG_WINDOW, 10,540);
        cv::namedWindow(DEBUG_WINDOW2);
        cv::moveWindow(DEBUG_WINDOW, 10,540);
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

    if (debug % 2 == 1)
        printf("Got a bbox with a corresponding depth image!\n");

    auto [cv_ptr_bgr, cv_ptr_depth] = images;

    /// >>> All the processing happens in getWorldPose!
    auto [success, module_world_pose] = getWorldPose(cv_ptr_bgr->image, cv_ptr_depth->image, bbox);
    /// <<<

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
Pose_extraction::getWorldPose(const cv::Mat &bgr_image, const cv::Mat &depth_image, const cv::Rect &bounding_box) {

    auto ret = geometry_msgs::PoseWithCovarianceStamped{};

    /// Do your magic here!!!
    // TODO: Magic!


    bool all_good{false};
    if (all_good) {
        return std::tuple<bool, geometry_msgs::PoseWithCovarianceStamped>(true, ret);
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
