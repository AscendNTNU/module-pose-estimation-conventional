//
// Created by marius on 09.04.2020.
//

#ifndef MODULE_POSE_ESTIMATION_DEPTHTOWORLD_H
#define MODULE_POSE_ESTIMATION_DEPTHTOWORLD_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <tuple>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "cvPointUtilities.h"

enum class GridpointDirection{y_neg, y_pos, x_neg_y_neg, x_neg_y_pos, x_pos_y_neg, x_pos_y_pos};

struct Gridpoint {
    cv::Point2i p;
    double dist;
    GridpointDirection dir;  // Direction in which to add new points
};

void printRot(const tf2::Quaternion& test_rotation);

std::vector<cv::Point3f>
findXYZ(const std::vector<cv::Point2f> &corner_points, const cv::Mat &depth_image, const std::vector<double> &depth_camera_info_K);

cv::Point3f findXYZ(const cv::Point2f &point, const cv::Mat &depth_image, const std::vector<double> &depth_camera_info_K);



/// Return the points all scaled with a factor "scaling" towards the center (scaling=1 gives only the center point)
std::vector<cv::Point2f> getScaledTowardsCenter(const std::vector<cv::Point2f>& points, double scaling);

/// Find the n closest points to point with valid depth data
std::vector<cv::Point3f> getNCloseValidPoints(const cv::Point2f& point, int n, const cv::Mat& depth_image);

/// Comparison function for close_points priority queue
bool distGreaterThan(const Gridpoint& a, const Gridpoint& b);

/// Add the relevant neighbors of next_point to the priority queue closest points
void addNeighbors(const cv::Point2f& source_point, const Gridpoint& neighbor_gridpoint,
                  std::priority_queue<Gridpoint, std::vector<Gridpoint>, decltype(&distGreaterThan)>& closest_points_queue);


std::tuple<cv::Point3f, tf2::Quaternion> getCenterAndRotation(const std::vector<cv::Point3f>& ordered_depth_points);


/**
 * @brief Finds the rotation of the plate given the ordered corner points
 *
 * The function explects ordered_points to be orderes as sorted in pose_extraction (clockwise starting upper left) and
 * ignores the point with index ignore_point_idx and assumes center is the center of the relevant points.
 * Note: This function defines the blue plate to have unit vectors e_x to the right, e_y down and e_z into the plate
 *
 * @see getCenterAndRotation
 * @param[in] ordered_points: Found xyz cornerpoins ordered clockwise from upper left
 * @param[in] ignore_point_idx: Index of point to ignore (as three points fully defines the rotation)
 * @param[in] center: The assumed center of the points. Can be center of the four, or of two valid points opposite of each other
 * @returns Quaternion: Rotation quaternion of the plate defined by the points, in relation to the coordinate system
 *
 */
tf2::Quaternion
getRotation(const std::vector<cv::Point3f> &ordered_points, int ignore_point_idx, const cv::Point3f &center);

/// Returns the index of the point with the worst fit to the plane (along z-axis), or -1 if max_off < second_max_off * thresh_factor + thresh_bias
int worstFitPointIndex(const std::vector<cv::Point3f>& points, float thresh_factor=1, float thresh_bias=0);

geometry_msgs::PoseWithCovarianceStamped
getCameraPoseWithCov(const std::vector<cv::Point3f> &depth_corner_points, int debug);

bool checkSquareness(const std::vector<cv::Point3f> &points, double scaling, int debug);

std::vector<cv::Point3f> getInnerCornerPoints(const cv::Mat &depth_image, const std::vector<cv::Point2f> &corner_points,
                                              const std::vector<double> &depth_camera_info_K_arr,
                                              const std::vector<double> &depth_camera_info_D, double scaling);


#endif //MODULE_POSE_ESTIMATION_DEPTHTOWORLD_H
