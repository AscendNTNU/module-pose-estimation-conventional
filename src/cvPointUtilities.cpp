//
// Created by marius on 29.04.2020.
//

#include <vector>
#include <opencv2/opencv.hpp>


#include "cvPointUtilities.h"

std::ostream& operator<<(std::ostream& os, const cv::Point3f& p) {
    os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<cv::Point3f>& p_vec) {
    for (const auto& p : p_vec) {
        os << "(" << p.x << ", " << p.y << ", " << p.z << "), ";
    }
    return os;
}

std::vector<cv::Point2f> operator+(std::vector<cv::Point2f> vec, const cv::Point2f& point)
{
    for (auto &vec_point : vec)
        vec_point += point;
    return vec;
};

cv::Point2f operator-(cv::Point2f a, const cv::Point2f& b) { a -= b; return a;}

cv::Point3f operator-(cv::Point3f a, const cv::Point3f& b) { a -= b; return a;}

std::vector<cv::Point3f> operator+(std::vector<cv::Point3f> lhs_vec, const cv::Point3f& rhs) {
    for (auto& lhs : lhs_vec)
        lhs+=rhs;
    return lhs_vec;
};

/*std::vector<cv::Point3f> operator+(const std::vector<cv::Point3i>& lhs_vec, const cv::Point3f& rhs) {
    std::vector<cv::Point3f> ret_vec(lhs_vec.size());
    for (int i{0}; i < lhs_vec.size(); ++i) {
        ret_vec.at(i) = lhs_vec.at(i) - rhs;
    }
    return ret_vec;
}*/

std::vector<cv::Point3f> operator-(const std::vector<cv::Point3f>& lhs_vec, const cv::Point3f& rhs){
    return lhs_vec + (-rhs);
};

std::vector<cv::Point3f>& operator+=(std::vector<cv::Point3f>& lhs_vec, const cv::Point3f& rhs) {
    for (auto& lhs_point : lhs_vec)
        lhs_point += rhs;
    return lhs_vec;
}

double lengthSqr(const cv::Point2f& p) {return p.x * p.x + p.y * p.y;}

double lengthSqr(const cv::Point3f& p) {return p.x * p.x + p.y * p.y + p.z * p.z;}

cv::Point2f getAverage(const std::vector<cv::Point2f> &points)
{
    cv::Point2f center{0, 0};

    for (const auto& point:  points) { center += point; }

    center *= (1 / static_cast<float>(points.size()));
    return center;
}

cv::Point3f getAverage(const std::vector<cv::Point3f> &points)
{
    cv::Point3f center{0, 0, 0};

    for (const auto& point:  points) { center += point; }

    center *= (1 / static_cast<float>(points.size()));
    return center;
}

cv::Point3f normalized(const cv::Point3f& p) {return p / std::sqrt(lengthSqr(p));};

cv::Point3f getCrossProd(const cv::Point3f& p1, const cv::Point3f& p2) {
    return cv::Point3f{p1.y*p2.z - p1.z*p2.y, p1.z*p2.x - p1.x*p2.z, p1.x*p2.y - p1.y*p2.x};}

bool comparePointsClockwise(cv::Point2f a, cv::Point2f b, const cv::Point2f &mean)
{
    a -= mean;
    b -= mean;

    // Center-point is defined as first
    if (b.x == 0 && b.y == 0)
        return false;
    if (a.x == 0 && a.y == 0)
        return true;

    if (b.y == 0)
    { // If b is on x-axis
        if (b.x < 0)
        { // If b is on y axis to left
            return false;
        }
        else
        { // If b is on y axis to right
            if (a.y == 0)
            {
                return a.x < 0;
            }
            else
            {
                return a.y < 0;
            }
        }
    }
    else if (a.y == 0)
    { // If a is on x-axis
        if (a.x < 0)
        { // If a is to the left
            return true;
        }
        else
        { // If a is to the right
            return b.y > 0;
        }
    }

    // They are not on the y-axis
    if (a.y < 0 && b.y > 0)
    {
        return true;
    }
    else if (a.y > 0 && b.y < 0)
    {
        return false;
    }
    // Then they are in the same half (upper/lower)
    return a.x / a.y > b.x / b.y;
}

void sortPointsClockwise(std::vector<cv::Point2f> &input_points)
{
    cv::Point2f center{0, 0};
    for (const auto& point : input_points)
    {
        center += point;
    }
    center.x /= input_points.size();
    center.y /= input_points.size();

    std::sort(input_points.begin(), input_points.end(),
              [&center](const cv::Point2f& a, const cv::Point2f& b) { return comparePointsClockwise(a, b, center); });
}

std::tuple<cv::Point3f, double, double> getPlaneFit(std::vector<cv::Point3f> depth_points, int debug) {
    /// See https://www.ilikebigbits.com/2015_03_04_plane_from_points.html
    /// Function assumes that the normal vector is not too far off the z-axis
    cv::Point3f center = getAverage(depth_points);
    depth_points += (-center);

    double S_xx{0}, S_yy{0};
    double S_xy{0}, S_xz{0}, S_yz{0};
    for (const auto& p : depth_points) {
        S_xx += p.x * p.x;
        S_yy += p.y * p.y;
        S_xy += p.x + p.y;
        S_xz += p.x * p.z;
        S_yz += p.y * p.z;
    }
    // The normal vector n is on the form [a, b, 1]. So the slope dz/dx == -a
    double D{S_xx * S_yy - S_xy * S_xy};
    double a{(S_yz * S_xy - S_xz * S_yy) / D};
    double b{(S_xz * S_xy - S_yz * S_xx) / D};

    if (D==0) {
        if (debug >= 0)
            printf("Points are on a line in getPlaneFit, D is 0.\n");
        a = b = 0;
    }

    return std::make_tuple(center, -a, -b);
}
