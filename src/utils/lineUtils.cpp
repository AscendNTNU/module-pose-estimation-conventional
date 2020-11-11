//
// Created by marius on 11.11.2020.
//

#include "lineUtils.h"

bool acceptLinePair(const cv::Vec2f &line1, const cv::Vec2f &line2, const float &minTheta)
{
    float theta1 = line1[1], theta2 = line2[1];

    if (theta1 >= CV_PI)
        theta1 -= CV_PI;
    if (theta2 >= CV_PI)
        theta2 -= CV_PI;

    if (std::min(abs(theta1 - theta2), abs(theta2 - theta1)) < minTheta)
        return false;

    if (theta1 < theta2)
    {
        return (theta1 + CV_PI - theta2) > minTheta;
    }
    else
    {
        return (theta2 + CV_PI - theta1) > minTheta;
    };
};

cv::Point2f computeIntersect(const cv::Vec2f &line1, const cv::Vec2f &line2, const cv::Point2f &offset)
{
    std::vector<cv::Point2f> p1 = lineToPointPair(line1);
    std::vector<cv::Point2f> p2 = lineToPointPair(line2);

    float denom = (p1[0].x - p1[1].x) * (p2[0].y - p2[1].y) - (p1[0].y - p1[1].y) * (p2[0].x - p2[1].x);
    cv::Point2f intersect(((p1[0].x * p1[1].y - p1[0].y * p1[1].x) * (p2[0].x - p2[1].x) -
                           (p1[0].x - p1[1].x) * (p2[0].x * p2[1].y - p2[0].y * p2[1].x)) /
                          denom +
                          offset.x,
                          ((p1[0].x * p1[1].y - p1[0].y * p1[1].x) * (p2[0].y - p2[1].y) -
                           (p1[0].y - p1[1].y) * (p2[0].x * p2[1].y - p2[0].y * p2[1].x)) /
                          denom +
                          offset.y);

    return intersect;
}

cv::Rect limitOuterCroppingRectangle(cv::Rect rect, const cv::Rect &inner_rect, const int &im_width, const int &im_height)
{
    // Expand outer bounding rectangle to contain inner bounding rectangle
    if (inner_rect.x < rect.x)
        rect.x = inner_rect.x;
    if (inner_rect.x + inner_rect.width > rect.x + rect.width)
        rect.width = inner_rect.x + inner_rect.width - rect.x;
    if (inner_rect.y < rect.y)
        rect.y = inner_rect.y;
    if (inner_rect.y + inner_rect.height > rect.y + rect.height)
        rect.height = inner_rect.y + inner_rect.height - rect.y;

    // Limit outer bounding rectangle to the size of the image
    if (rect.x < 0)
    {
        rect.width += rect.x;
        rect.x = 0;
    }
    if (rect.x + rect.width > im_width)
    {
        rect.width = im_width - rect.x;
    }
    if (rect.y < 0)
    {
        rect.height += rect.y;
        rect.y = 0;
    }
    if (rect.y + rect.height > im_height)
    {
        rect.height = im_height - rect.y;
    }
    return rect;
}

std::vector<cv::Point2f> lineToPointPair(const cv::Vec2f &line)
{
    std::vector<cv::Point2f> points{2};

    float r = line[0], t = line[1];
    double cos_t = cos(t), sin_t = sin(t);
    double x0 = r * cos_t, y0 = r * sin_t;
    double alpha = 1000;

    points[0] = cv::Point2f(x0 + alpha * (-sin_t), y0 + alpha * cos_t);
    points[1] = cv::Point2f(x0 - alpha * (-sin_t), y0 - alpha * cos_t);

    return points;
}

void linepointsToRadiusAngle(const std::vector<cv::Vec4i> &lines_points_in, std::vector<cv::Vec2f> &lines_out)
{
    cv::Vec2f line;
    for (auto &pts : lines_points_in)
    {
        // pts is on the form [pt1.x, pt1.y, pt2.x, pt2.y]

        double theta, r, a, b; // Assuming the line is on the form y=ax+b
        if (pts[0] - pts[2] == 0)
        {
            // Handling the case of a vertical line
            theta = 0;
            r = pts[0];
        }
        else if (pts[1] - pts[3] == 0)
        {
            theta = CV_PI / 2;
            r = pts[1];
        }
        else
        {
            a = (pts[3] - pts[1]) / static_cast<double>(pts[2] - pts[0]); // a = (y_1 - y_0) / (x_1 - x_0)
            b = pts[3] - a * pts[2];                                      // b = y_1 - a*x_1
            theta = std::atan(-1 / a);
            r = std::sin(theta) * b;
        }

        line[0] = r;
        line[1] = theta;
        lines_out.emplace_back(line);
    }
}

std::vector<cv::Point2f>
computeMultiIntersections(const std::vector<cv::Vec2f> &lines, const float &minAngleDiff,
                          const cv::Point2f &offset, const int &max_intersects)
{
    std::vector<cv::Point2f> intersections{};
    int remaining_intersects{max_intersects};

    // Loop thorugh each line pair once
    for (unsigned int i = 0; i < lines.size(); i++)
    {
        for (unsigned int j = 0; j < i; j++)
        {
            if (acceptLinePair(lines[i], lines[j], minAngleDiff)) // Reject the lines if they are too parallel
            {
                intersections.push_back(computeIntersect(lines[i], lines[j], offset));

                if (--remaining_intersects == 0)
                    return intersections;
            }
        }
    }

    return intersections;
}

