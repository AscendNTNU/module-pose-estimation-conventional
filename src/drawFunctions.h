//
// Created by marius on 05.03.2020.
//

#ifndef MODULE_POSE_ESTIMATION_DRAWFUNCTIONS_H
#define MODULE_POSE_ESTIMATION_DRAWFUNCTIONS_H

#include <opencv2/opencv.hpp>
#include "dbscan.h"

/**
 * Draw lines on image.
 *
 * Draws a maximum number of 'max_lines' lines. Can draw the lines with an offset along x and y.
 *
 * @param image: Image to draw on.
 * @param lines: Lines to draw (on raduis, angle - format)
 * @param max_lines: Maximum number of lines to draw
 * @param offset: Draw the lines with an offset origin
 */

/// Return a vector of n different colors up to n~=13, the rest is white.
std::vector<cv::Scalar> getNColors(int n);


void drawLines(cv::Mat& image, const std::vector<cv::Vec2f>& lines, int max_lines=100,
               const cv::Point2f& offset=cv::Point2f{0, 0});

/// Draw the clustered points from DBSCAN on image (with offset if provided)
void drawClusteredPoints(cv::Mat& image, const DBSCAN& clustering, const cv::Point2i& offset=cv::Point{0, 0});

/// Draw lines between points in a vector, connecting the last to the first point

void drawCycle(cv::Mat& image, const std::vector<cv::Point2f>& points, const cv::Point2f& offset=cv::Point{0, 0},
               const cv::Scalar& color = cv::Scalar{0, 128, 256}, bool arrowed=false);

/**
 * @brief Draw points on an image
 *
 *
 *
 * @see computeMultiIntersections
 *
 * @param[in] image_draw: Image to draw on.
 * @param[out] points: A vector of the intersections found.
 * @param offset: Draw the lines with an offset origin
 */
void drawPoints(cv::Mat &image_draw, std::vector<cv::Point2f> &points,
                const cv::Point2f &offset, const cv::Scalar& color=cv::Scalar(0, 191, 255));
void drawPoints(cv::Mat &image_draw, std::vector<cv::Point> &points,
                const cv::Point2i &offset, const cv::Scalar& color=cv::Scalar(0, 191, 255));



#endif //MODULE_POSE_ESTIMATION_DRAWFUNCTIONS_H
