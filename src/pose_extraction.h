//
// Created by marius on 07.11.2019.
//

#ifndef MODULE_POSE_ESTIMATION_POSE_EXTRACTION_H
#define MODULE_POSE_ESTIMATION_POSE_EXTRACTION_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vision_msgs/Detection2D.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <ros/console.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "utils/drawFunctions.h"
#include "utils/cvPointUtilities.h"
#include "utils/LookupQueue/LookupQueue.h"
#include "utils/lineUtils.h"


/// Small exception class for throwing "NotImplemented"-errors
class NotImplementedError : public std::logic_error
{
public:
    NotImplementedError() : std::logic_error("Function not yet implemented") { };
};

static const std::string INPUT_WINDOW = "Image window";     ///< Debug window name
static const std::string DEBUG_WINDOW = "Debug window";     ///< Debug window name
static const std::string DEBUG_WINDOW2 = "Debug2 window";   ///< Debug window name


// The overall object managing the Pose_extraction process
class Pose_extraction {
private:
    /// Nodehandle for this node
    ros::NodeHandle nh_;
    /// Image transport for subscription
    image_transport::ImageTransport it_;

    tf2_ros::Buffer tf_buffer{ros::Duration{30}};  // TODO: Reduce buffer size when not debuging
    tf2_ros::TransformListener tf_listener{tf_buffer};   ///< Transform listener

    /// Vars for use in functions, (hopefully) improving memory (de/)allocation time
    cv::Mat smoothed_depth_image;
    cv::Mat depth_mask;
    cv::Mat debug_window;

    /// Try to import the image given by the message. If it fails: return false
    static bool importImageBgr(const sensor_msgs::ImageConstPtr &msg, cv_bridge::CvImagePtr& cv_ptr_out);
    /// Try to import the image given by the message. If it fails: return false
    static bool importImageDepth(const sensor_msgs::ImageConstPtr &msg, cv_bridge::CvImagePtr& ptr_out);
    /// Try to transform bbox detection to cv::Rect. If it fails: return false
    static bool importBboxRect(const vision_msgs::Detection2D &bbox_msg, cv::Rect &rect_out);


    // Core function finding pose
    std::tuple<bool, geometry_msgs::PoseWithCovarianceStamped> getWorldPose(const cv::Mat &bgr_image, const cv::Mat& depth_image,
                                                                            const cv::Rect& bounding_box);

    /**
     * @brief Return cornerpoints of the square in camera pixels
     *
     * Will find some more exact points for the corners of the square, and return them in "cornerpoints_out".
     *
     * @param[in] cv_color_image: Full input image (not cropped) to find features of
     * @param[in] blueness_image: Blueness grayscale image, from bluenessImageMasked (via getBoundingRectangle)
     * @param[in] inner_bounding_rect: The corners of the blue square (in the image) from the featureDetection algorithm (sift).
     * @param[in] outer_bounding_rect: The bounding rectangle from getBoundingRectangle in which the box lies.
     * @returns bool: Whether the score of "cornerPointScore" is high enough for the result to be valid
     */



    typedef std::pair<cv_bridge::CvImagePtr, cv_bridge::CvImagePtr> image_ptr_tuple;
    LookupQueue<ros::Time, image_ptr_tuple> image_ptr_buffer;


    /// Transform the PoseWithCovStamped to worldspace (with changing metadata)
    bool transformToWorld(geometry_msgs::PoseWithCovarianceStamped &pose_with_cov_stamped,
                          const std_msgs::Header &from_header);


public:
    int debug{0};  ///< Whether to use debug mode (>= 1). Odd numbers will print timestamps. See launc file

    Pose_extraction(ros::NodeHandle &nh, image_transport::ImageTransport &it, int img_buffer_size);
    ~Pose_extraction();

    ros::Publisher pose_publisher;  ///< Publisher variable

    // Callback on images (saving images in image_msg_buffer)
    void imageCb(const sensor_msgs::ImageConstPtr &bgr_msg, const sensor_msgs::ImageConstPtr &depth_msg);

    // Callback on bounding box
    void bboxCb(const vision_msgs::Detection2D& bbox_msg);

    // Callback to get depth camera info
    void depthCameraInfoCb(const boost::shared_ptr<sensor_msgs::CameraInfo const>& ptr_camera_info_message);

    // Depth mask variables
    int depth_dilation_kernel_size{3};  // 20 is a nice default value for approximately 2m
    int mask_dilation_kernel_size{3};  // 20 is a nice default value for approximately 2m
    int depth_mean_offset_value{0};  // 20 is a nice default value for approximately 2m

    // Depth camera info variables. See Pose_exctraction::depthCameraInfoCb and pose_extraction::findPoseAndCov
    bool has_depth_camera_info{false};
    cv::Mat depth_camera_info_K{3, 3, CV_32FC1};  // Camera matrix of depth camera
    std::vector<double> depth_camera_info_K_vec{0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<double> depth_camera_info_D;  // Distortion coeffitients of depth camera

    std::string world_tf_frame_id;  // frame_id of world frame
};



/**
 * @brief Get a grayscale image of the 'blueness' of the input image. Brighter output -> more fitting.
 *
 * Overloaded  adding the depth_mask parameter.
 * There are multiple methods for determining how close the image pixels
 * are to the correct color, all listed in the IMG_CONVERSION class. The depth_mask will
 * signify what areas to set to 0 in output.
 *
 * @param[in] im_in_bgr: The input image to determine the blueness of.
 * @param[out] im_grey_out: The output grayscale image of the blueness of the image.
 * @param[in] conv_type: The image conversion type. Choose different methods by choosing different IMG_CONVERSION enums.
 * @param[in] depth_mask: Mask with areas to ignore (in black). Typically based on depth.
 * @param[in] blur: Whether to blur the result before returning. default: true
 */
void bluenessImageMasked(const cv::Mat &im_in_bgr, cv::Mat &im_grey_out, const cv::Mat &depth_mask, const bool &blur);

/**
 * @brief Get a grayscale image of the 'blueness' of the input image. Brighter output -> more fitting.
 *
 * There are multiple methods for determining how close the image pixels
 * are to the correct color, all listed in the IMG_CONVERSION class.
 *
 * @param[in] im_in_bgr: The input image to determine the blueness of.
 * @param[out] im_grey_out: The output grayscale image of the blueness of the image.
 * @param[in] conversion_type: The image conversion type. Choose different methods by choosing different IMG_CONVERSION enums.
 * @param[in] blur: Whether to blur the result before returning. default: true
 */
void bluenessImage(const cv::Mat &im_in_bgr, cv::Mat &im_grey_out, const bool &blur);

#endif //MODULE_POSE_ESTIMATION_POSE_EXTRACTION_H
