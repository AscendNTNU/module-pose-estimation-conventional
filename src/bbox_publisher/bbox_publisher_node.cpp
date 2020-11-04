//
// Created by marius on 07.11.2019.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "Bbox_publisher.h"


int main(int argc, char** argv) {
    ROS_INFO("Running bbox_publisher_node, dummy node");

    ros::init(argc, argv, "bbox_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it{nh};

    /// Initialize subscriber/publisher adress parameters, retrieve params
    ros::NodeHandle private_nh("~");
    std::string itopic_image, itopic_depth_image, otopic_detection2D;
    int depth_dilation_kernel_size, mask_dilation_kernel_size, depth_mean_offset;

    private_nh.param<int>("depth_mean_offset_value",depth_mean_offset, 0);
    private_nh.param<int>("depth_dilation_kernel_size", depth_dilation_kernel_size, 3);
    private_nh.param<int>("mask_dilation_kernel_size", mask_dilation_kernel_size, 3);

    private_nh.param<std::string>("itopic_image", itopic_image, "/videofile/image_raw");  // Get video: "/videofile/image_raw", webcam: /data/ZED_camera
    private_nh.param<std::string>("itopic_depth_image", itopic_depth_image, "/videofile/depth_raw");  // Get video: "/videofile/image_raw", webcam: /data/ZED_camera

    private_nh.param<std::string>("otopic_detection2D", otopic_detection2D, "/perception/bbox_dummy");


    // Set up extractor object and parameters
    Bbox_publisher bboxPublisher{nh, it};  // TODO
    bboxPublisher.depth_mean_offset_value = depth_mean_offset;
    bboxPublisher.depth_dilation_kernel_size = depth_dilation_kernel_size;
    bboxPublisher.mask_dilation_kernel_size = mask_dilation_kernel_size;

    /// Set up subscribers and publishers
    image_transport::SubscriberFilter rgb_image_sub(it, itopic_image, 1);
    image_transport::SubscriberFilter depth_image_sub(it, itopic_depth_image, 1);

    ros::Publisher bbox_publisher =
            nh.advertise<vision_msgs::Detection2D>(otopic_detection2D, 10);
    bboxPublisher.bbox_publisher = bbox_publisher;


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> sync_policy;

    message_filters::Synchronizer<sync_policy> image_syncer(sync_policy(10), rgb_image_sub, depth_image_sub);
    image_syncer.registerCallback(boost::bind(&Bbox_publisher::imageCb, &bboxPublisher, _1, _2));


    ros::spin();
    return 0;
}
