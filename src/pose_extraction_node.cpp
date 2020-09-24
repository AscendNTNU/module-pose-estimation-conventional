//
// Created by marius on 07.11.2019.
//

#include <ros/ros.h>


#include <sensor_msgs/CameraInfo.h>

#include "pose_extraction.h"


int main(int argc, char** argv) {
    ROS_INFO("Running pose_extraction_node");


    ros::init(argc, argv, "pose_extraction");
    ros::NodeHandle nh;
    image_transport::ImageTransport it{nh};

    /// Initialize subscriber/publisher adress parameters, retrieve params
    ros::NodeHandle private_nh("~");
    std::string itopic_image, itopic_depth_image, itopic_bounding_box, itopic_depth_camera_info,
    itopic_model_tf_frame_id, otopic_PoseWithCovarianceStamped, otopic_extracted_tf_frame_id,
    world_tf_frame_id;
    int depth_dilation_kernel_size, mask_dilation_kernel_size, depth_mean_offset;

    private_nh.param<int>("depth_mean_offset_value",depth_mean_offset, 0);
    private_nh.param<int>("depth_dilation_kernel_size", depth_dilation_kernel_size, 3);
    private_nh.param<int>("mask_dilation_kernel_size", mask_dilation_kernel_size, 3);

    private_nh.param<std::string>("itopic_image", itopic_image, "/videofile/image_raw");  // Get video: "/videofile/image_raw", webcam: /data/ZED_camera
    private_nh.param<std::string>("itopic_depth_image", itopic_depth_image, "/videofile/depth_raw");  // Get video: "/videofile/image_raw", webcam: /data/ZED_camera
    private_nh.param<std::string>("itopic_depth_camera_info", itopic_depth_camera_info, "/d435i/depth/camera_info");
    private_nh.param<std::string>("world_tf_frame_id", world_tf_frame_id, "None");

    private_nh.param<std::string>("otopic_PoseWithCovarianceStamped", otopic_PoseWithCovarianceStamped, "/perception/modulePoseWithCovariance");


    // Set up extractor object and parameters
    Pose_extraction extractor{nh, it};  // TODO
    extractor.depth_mean_offset_value = depth_mean_offset;
    extractor.depth_dilation_kernel_size = depth_dilation_kernel_size;
    extractor.mask_dilation_kernel_size = mask_dilation_kernel_size;
    extractor.world_tf_frame_id = world_tf_frame_id;

    /// Set up subscribers and publishers
    image_transport::SubscriberFilter rgb_image_sub(it, itopic_image, 1);
    image_transport::SubscriberFilter depth_image_sub(it, itopic_depth_image, 1);
    ros::Subscriber depth_info_sub = private_nh.subscribe(itopic_depth_camera_info, 1,
                                                          &Pose_extraction::depthCameraInfoCb, &extractor);

    ros::Publisher pose_publisher =
            nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(otopic_PoseWithCovarianceStamped, 100);
    extractor.pose_publisher = pose_publisher;


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> sync_policy;

    message_filters::Synchronizer<sync_policy> image_syncer(sync_policy(10), rgb_image_sub, depth_image_sub);
    image_syncer.registerCallback(boost::bind(&Pose_extraction::imageCb, &extractor, _1, _2));


    ros::spin();
    return 0;
}
