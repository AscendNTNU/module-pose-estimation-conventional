//
// Created by marius on 21.01.2020.
//

#include "blue_square_score.h"
#include <ros/package.h>


double blueSquareScore::getBlueSquareScore(const cv::Mat &image_in,const Point_<float> *dst_quad) {
    
    homography = cv::getPerspectiveTransform(dst_quad, src_quad);

    cv::warpPerspective(image_in, transformed_image, homography, QueryImage.size());
    
    cv::absdiff(QueryImage,transformed_image,absdiff_img);

    average = cv::mean(absdiff_img);
    
    score = average[0] + average[1] + average[2] + std::max(std::max(average[0], average[1]), average[2]) - std::min(std::min(average[0], average[1]), average[2]);
    
    score = 1-0.000001*score*score*score;
    
    if (score < 0) {score = 0;}
    
    return score;
}

blueSquareScore::blueSquareScore() {
	std::string path_to_bsxml = ros::package::getPath("module-pose-estimation-conventional") + "/src/BSbluered.xml";
	cv::FileStorage fsbss(path_to_bsxml, cv::FileStorage::READ);
	fsbss["blueredMatrix"] >> QueryImage;
	fsbss.release(); 
    
    src_quad[0] =  cv::Point2f(0, 0);
    src_quad[1] =  cv::Point2f((float)QueryImage.cols, 0);
    src_quad[2] =  cv::Point2f((float)QueryImage.cols, (float)QueryImage.rows);
    src_quad[3] =  cv::Point2f(0, (float)QueryImage.rows);

}
