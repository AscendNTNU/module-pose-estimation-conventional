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

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <ros/console.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "blue_square_score.h"
#include "foreground_extraction.h"
#include "utils/drawFunctions.h"
#include "depthToWorld.h"
#include "utils/cvPointUtilities.h"


/// Small exception class for throwing "NotImplemented"-errors
class NotImplementedError : public std::logic_error
{
public:
    NotImplementedError() : std::logic_error("Function not yet implemented") { };
};

static const std::string INPUT_WINDOW = "Image window";     ///< Debug window name
static const std::string RESULT_WINDOW = "Blueness window";   ///< Debug window name
static const std::string DEPTH_WINDOW = "Canny on blue window";     ///< Debug window name
static const std::string DEPTH_MASK_WINDOW = "Depth mask window";   ///< Debug window name


// The overall object managing the Pose_extraction process
class Pose_extraction {
private:
    /// Nodehandle for this node
    ros::NodeHandle nh_;
    /// Image transport for subscription
    image_transport::ImageTransport it_;

    tf2_ros::TransformBroadcaster tf_broadcaster;   ///< Unused broadcasting variable
    geometry_msgs::TransformStamped tf_pub;         ///< Unused publishing variable

    /// Feature detector class. For use with doSiftSwitch
    blueSquareScore BlueSquareScoreCalculator;

    tf2_ros::Buffer tf_buffer{ros::Duration{30}};  // TODO: Reduce buffer size when not debuging
    tf2_ros::TransformListener tf_listener{tf_buffer};   ///< Transform listener

    /// Vars for use in functions, (hopefully) improving memory (de/)allocation time
    cv::Mat smoothed_depth_image;
    cv::Mat depth_mask;
    cv::Mat debug_window;

    /// Image to work on, maybe saving some memory (de)alloccation time.
    cv::Mat cv_work_image;


    /// Try to import the image given by the message. If it fails: return false
    static bool importImageBgr(const sensor_msgs::ImageConstPtr &msg, cv_bridge::CvImagePtr& cv_ptr_out);
    /// Try to import the image given by the message. If it fails: return false
    static bool importImageDepth(const sensor_msgs::ImageConstPtr &msg, cv_bridge::CvImagePtr& ptr_out);

    /**
     * @brief Find an outer bounding box that will at least contain the blue plate.
     *
     * The task is in essence to crop the image to a smaller area. This is so that Torleifs sift/orb feature detector
     * can work quickly on a smaller part of the image, and to work as a "narrowing down" algorithm to work further
     * on the image cropped by the bounding rectangle.
     *
     * @param[in] cv_im_draw_on: The input image in which to determine where the blue square is.
     * @param[out] blueness_image: A copy of the cv_ptr_in.image, made to be modified and worked on. Should return the image from bluenessImageMasked
     * @param[out] bounding_rectangle: The bounding rectangle in which the blue square lies. Returns cv::Rect{0, 0, 0, 0} when nothing is found.
     */
    std::tuple<cv::Rect, cv::Rect>
    getBoundingRectangle(const cv::Mat &blueness_image); //TODO: Add const to input


    /**
     * @brief A function that chooses the different sift/orb/featuredetection methods (or none of them)
     *
     * @param[out] points_out: A vector of points corresponding to the corners of the inner bounding rectangle.
     * @param[in] bounding_rect: The bounding rectangle in which the blue square is. Used for cropping later.
     */
    void getInnerBoundingRectangle(const cv::Rect &bounding_rect, std::vector<cv::Point2f> &points_out);

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
    std::vector<cv::Point2f> findCornerPoints(Mat &cv_color_image, const cv::Mat &blueness_image,
                                              const cv::Rect &inner_bounding_rect,
                                              cv::Rect outer_bounding_rect); // TODO: Add TF to I/O of function


public:
    // TODO: Make all debug statements check with this debug code (0 meaning no debug)
    int debug{0};  ///< Whether to use debug mode (>= 1). Odd numbers will print timestamps

    Pose_extraction(ros::NodeHandle &nh, image_transport::ImageTransport &it);
    ~Pose_extraction();

    ros::Publisher pose_publisher;  ///< Object specific publisher variable

    void imageCb(const sensor_msgs::ImageConstPtr &bgr_msg, const sensor_msgs::ImageConstPtr &depth_msg);

    // Callback to get depth camera info
    // TODO: Find a way to get this info once without a callback with flag
    void depthCameraInfoCb(const boost::shared_ptr<sensor_msgs::CameraInfo const>& ptr_camera_info_message);

    // Depth mask variables
    int depth_dilation_kernel_size{3}; // 20 is a nice default value for approximately 2m
    int mask_dilation_kernel_size{3}; // 20 is a nice default value for approximately 2m
    int depth_mean_offset_value{0}; // 20 is a nice default value for approximately 2m

    // Depth camera info variables. See Pose_exctraction::depthCameraInfoCb and pose_extraction::findPoseAndCov
    bool has_depth_camera_info{false};
    cv::Mat depth_camera_info_K{3, 3, CV_32FC1};  // Camera matrix of depth camera
    std::vector<double> depth_camera_info_K_arr{0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<double> depth_camera_info_D;  // Distortion coeffitients of depth camera

    std::string world_tf_frame_id;  // frame_id of world frame

    double cornerPointScore(const Mat &image_in, const std::vector<cv::Point2f> &corners_in);

    /// Transform the PoseWithCovStamped to worldspace (with changing metadata)
    bool transformToWorld(geometry_msgs::PoseWithCovarianceStamped &pose_with_cov_stamped,
                          const std_msgs::Header &from_header);
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

//TODO: Move these tools to another .h/.cpp file
/**
 * @brief Check whether the angle between two lines is smaller than minTheta
 *
 * Returns true if the angle is bigger, false if they are too parallel.
 *
 *
 * @param[in] line1, line2: The two lines for angle comparison
 * @param[in] minTheta: Minimum angle for acceptance
 * @returns bool: true if the angle between lines is sufficiently large, false if they are too parallel
 */
bool acceptLinePair(const cv::Vec2f& line1, const cv::Vec2f& line2, const float& minTheta);

/**
 * @brief Computes the intersection of all the lines defines by an angle and a radius, with a minimum angle difference between lines.
 *
 * @see acceptLinePair
 * @see computeIntersect
 * @see Pose_extractor::drawPoints
 *
 * @param[in] lines: The lines that one shall compute the intersection of, on the form [radius, angle]
 * @param[in] offset: Give an offset to the line intersect coordinates
 * @returns intersections: The intersection points of all the lines
 */
std::vector<cv::Point2f>
computeMultiIntersections(const std::vector<cv::Vec2f> &lines, const float &minAngleDiff,
                          const cv::Point2f& offset=cv::Point2f{0, 0}, const int &max_intersects = -1);

/**
 * @brief Computes the intersection of two lines defined by an angle and radius.
 *
 * @see computeMultiIntersections
 *
 * @param[in] line1, line2: The lines that one shall compute the intersection of
 * @param[in] offset: Give an offset to the line intersect coordinates
 * @returns intersection: The intersection point of the two lines
 *
 */
cv::Point2f computeIntersect(const cv::Vec2f &line1, const cv::Vec2f &line2, const cv::Point2f& offset=cv::Point2f{0, 0});

/**
 * @brief Get a point pair from the line defined by an angle and a radius
 *
 * @param[in] line: Line to find points for
 * @returns vector with two points defining the line.
 */
std::vector<cv::Point2f> lineToPointPair(const cv::Vec2f& line);

/**
 * @brief Get a vector of radius and angle from a vector of lines defined by two points.
 *
 * @param[in] lines_points_in: Vector of 4-point vectors defining a line
 * @returns lines_out: vector of 2-point vector definint the radii and angles of the lines.
 */
void linepointsToRadiusAngle(const std::vector<cv::Vec4i>& lines_points_in, std::vector<cv::Vec2f>& lines_out);

/// Return a string with the type of the matrix. Input cv::Mat.type()
std::string matType2str(int type);  // Debugging tool

/// Make sure the cropping rectangle rect is within the constraints of an image with im_width and im_height
cv::Rect limitOuterCroppingRectangle(cv::Rect rect, const cv::Rect &inner_rect, const int &im_width, const int &im_height);

#endif //MODULE_POSE_ESTIMATION_POSE_EXTRACTION_H
