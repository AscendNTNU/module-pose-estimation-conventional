//
// Created by marius on 30.10.2020.
//

#ifndef MODULE_POSE_ESTIMATION_CONVENTIONAL_BBOX_PUBLISHER_H
#define MODULE_POSE_ESTIMATION_CONVENTIONAL_BBOX_PUBLISHER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <vision_msgs/Detection2D.h>

#include "../foreground_extraction.h"

class Bbox_publisher {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    cv::Mat smoothed_depth_image;
    cv::Mat depth_mask;

    void publishDetection(const cv::Rect &bounding_rect, const std_msgs::Header &header);
public:
    int debug;
    Bbox_publisher(ros::NodeHandle &nh, image_transport::ImageTransport &it);

    void imageCb(const sensor_msgs::ImageConstPtr &bgr_msg, const sensor_msgs::ImageConstPtr &depth_msg);

    // Depth mask variables
    int depth_dilation_kernel_size{3}; // 20 is a nice default value for approximately 2m
    int mask_dilation_kernel_size{3}; // 20 is a nice default value for approximately 2m
    int depth_mean_offset_value{0}; // 20 is a nice default value for approximately 2m

    ros::Publisher bbox_publisher;
};

/// Try to import the image given by the message. If it fails: return false
static bool importImageBgr(const sensor_msgs::ImageConstPtr &msg, cv_bridge::CvImagePtr& cv_ptr_out);
/// Try to import the image given by the message. If it fails: return false
static bool importImageDepth(const sensor_msgs::ImageConstPtr &msg, cv_bridge::CvImagePtr& ptr_out);

std::tuple<cv::Rect, cv::Rect> getBoundingRectangle(const cv::Mat &blueness_image); //TODO: Add const to input

void bluenessImageMasked(const cv::Mat &im_in_bgr, cv::Mat &im_grey_out, const cv::Mat &depth_mask, const bool &blur);

void bluenessImage(const cv::Mat &im_in_bgr, cv::Mat &im_grey_out, const bool &blur);

#endif //MODULE_POSE_ESTIMATION_CONVENTIONAL_BBOX_PUBLISHER_H
