//
// Created by marius on 29.04.2020.
//

#ifndef MODULE_POSE_ESTIMATION_CVPOINTUTILITIES_H
#define MODULE_POSE_ESTIMATION_CVPOINTUTILITIES_H


std::ostream& operator<<(std::ostream& os, const cv::Point3f& p);

std::ostream& operator<<(std::ostream& os, const std::vector<cv::Point3f>& p_vec);

std::vector<cv::Point2f> operator+(std::vector<cv::Point2f> vec, const cv::Point2f& point);

cv::Point2f operator-(cv::Point2f a, const cv::Point2f& b);

cv::Point3f operator-(cv::Point3f a, const cv::Point3f& b);

// std::vector<cv::Point3f> operator+(const std::vector<cv::Point3i>& lhs_vec, const cv::Point3f& rhs);

std::vector<cv::Point3f> operator+(std::vector<cv::Point3f> lhs_vec, const cv::Point3f& rhs);

std::vector<cv::Point3f> operator-(const std::vector<cv::Point3f>& lhs_vec, const cv::Point3f& rhs);

std::vector<cv::Point3f>& operator+=(std::vector<cv::Point3f>& lhs_vec, const cv::Point3f& rhs);

double lengthSqr(const cv::Point2f& p);

double lengthSqr(const cv::Point3f& p);

cv::Point2f getAverage(const std::vector<cv::Point2f> &points);

cv::Point3f getAverage(const std::vector<cv::Point3f> &points);

/// Return the normalized vector
cv::Point3f normalized(const cv::Point3f& p);

/// Return the cross product of two vectors
cv::Point3f getCrossProd(const cv::Point3f& p1, const cv::Point3f& p2);


/// Sort the points in-place. sorted in clockwise direction with the sorting starting directly to the left of the mean of the poins (neg. x-direction)
void sortPointsClockwise(std::vector<cv::Point2f>& input_points);

/// Compare two points (as with operator< ). See sortPoints to compare points in clockwise direction with (-1, 0) as firs
bool comparePointsClockwise(cv::Point2f a, cv::Point2f b, const cv::Point2f& mean);

/// Fits a plane to the input points that goes through the average of the points
/// Implementation of https://www.ilikebigbits.com/2015_03_04_plane_from_points.html
std::tuple<cv::Point3f, double, double> getPlaneFit(std::vector<cv::Point3f> depth_points);



#endif //MODULE_POSE_ESTIMATION_CVPOINTUTILITIES_H
