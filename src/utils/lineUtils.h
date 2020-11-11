//
// Created by marius on 11.11.2020.
//

#ifndef MODULE_POSE_ESTIMATION_CONVENTIONAL_LINEUTILS_H
#define MODULE_POSE_ESTIMATION_CONVENTIONAL_LINEUTILS_H

#include <vector>
#include <opencv2/core.hpp>

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

/// Make sure the cropping rectangle rect is within the constraints of an image with im_width and im_height
cv::Rect limitOuterCroppingRectangle(cv::Rect rect, const cv::Rect &inner_rect, const int &im_width, const int &im_height);


#endif //MODULE_POSE_ESTIMATION_CONVENTIONAL_LINEUTILS_H
