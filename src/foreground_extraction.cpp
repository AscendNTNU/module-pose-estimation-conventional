#include "foreground_extraction.h"

cv::Mat dilate_depth_image(const cv::Mat &depth_image, const int &dilation_kernel_size)
{
    cv::Mat temp;
    cv::Mat kernel_element = cv::getStructuringElement (0, cv::Size{2*dilation_kernel_size+1, 2*dilation_kernel_size+1}, cv::Point{dilation_kernel_size,dilation_kernel_size});
    cv::dilate(depth_image, temp, kernel_element);

    return temp;
}

cv::Mat generate_foreground_mask(const cv::Mat &depth_image, const int &depth_mean_offset_value, const int &mask_dilation_kernel_size)
{
    cv::Mat temp = cv::Mat::zeros(depth_image.size(), CV_8UC1);
    depth_image.convertTo(temp, CV_8UC1, 1/256.0);
    double mean_depth_value = cv::mean(temp)[0];
    cv::threshold(temp,temp, mean_depth_value + depth_mean_offset_value, 255, cv::THRESH_BINARY_INV);
    cv::Mat kernel_element = cv::getStructuringElement (0, cv::Size{2*mask_dilation_kernel_size+1, 2*mask_dilation_kernel_size+1}, cv::Point{mask_dilation_kernel_size,mask_dilation_kernel_size});
    cv::Mat temp_dilated;
    cv::dilate(temp, temp_dilated, kernel_element);



    return temp_dilated;

}

