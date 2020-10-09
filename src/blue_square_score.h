//
// Created by marius on 21.01.2020.
//

#ifndef MODULE_POSE_ESTIMATION_BLUE_SQUARE_SCORE_H
#define MODULE_POSE_ESTIMATION_BLUE_SQUARE_SCORE_H


#include <opencv2/core/core.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <algorithm>


using namespace cv;
using namespace std; //TODO: remove all namespaces, but will result in errors at the current time.

class blueSquareScore {
private:

    cv::Mat QueryImage, homography, transformed_image, absdiff_img;
    cv::Point2f src_quad[4];
    cv::Scalar average;
    double score;

public:
    blueSquareScore();
    double getBlueSquareScore(const cv::Mat &image_in, const Point_<float> *dst_quad);
};


#endif //MODULE_POSE_ESTIMATION_BLUE_SQUARE_SCORE_H
