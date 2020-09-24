//
// Created by marius on 05.03.2020.
//

#include "drawFunctions.h"


std::vector<cv::Scalar> getNColors(int n) {
    static std::vector<cv::Scalar> COLOR_VECTOR{
            // Black
            cv::Scalar{0, 0, 0},
            // Reddish-grey, unused because the clusters are 1-indexed and "no-cluster" has index -1, so idx+1 is accessed.
            cv::Scalar{128, 128, 180},
            // Red
            cv::Scalar{0, 0, 255},
            // Lime
            cv::Scalar{0, 255, 0},
            // Blue
            cv::Scalar{255, 0, 0},
            // Yellow
            cv::Scalar{0, 255, 255},
            // Cyan
            cv::Scalar{255, 255, 0},
            // Magenta
            cv::Scalar{255, 0, 255},
            // Maroon
            cv::Scalar{0, 0, 128},
            // Olive (dark yellow)
            cv::Scalar{0, 128, 128},
            // Saddle brown
            cv::Scalar{19, 69, 139},
            // Teal
            cv::Scalar{128, 128, 0},
            // White
            cv::Scalar{255, 255, 255}

    };
    std::vector<cv::Scalar> ret;
    // Fill the first elements with colors, as long as possible
    for (unsigned long i{0}; i < COLOR_VECTOR.size(); ++i) {
        ret.emplace_back(COLOR_VECTOR.at(i));
    }

    // And the rest with white
    for (unsigned long i = COLOR_VECTOR.size(); i < n; ++i) {
        ret.emplace_back(COLOR_VECTOR.back());
    }
    return ret;
}


void drawLines(cv::Mat &image, const std::vector<cv::Vec2f> &lines, int max_lines,
               const cv::Point2f& offset) {

    /// Lines must be represented with a radius and an angle, like the format from cv::hughLines
    for (auto &line : lines) {

        // Ignore the line if it is too far off the cartesian axes:
        //float theta_m = std::fmod(line[1], CV_PI / 2);
        //if (CV_PI / 8 <= theta_m && theta_m <= 3 * CV_PI / 8) continue;

        float rho = line[0], theta = line[1];
        cv::Point pt1, pt2;
        double a = std::cos(theta), b = std::sin(theta);
        double x0 = a * rho, y0 = b * rho;
        pt1.x = cvRound(x0 + 2000 * (-b)) + offset.x;
        pt1.y = cvRound(y0 + 2000 * ( a)) + offset.y;
        pt2.x = cvRound(x0 - 2000 * (-b)) + offset.x;
        pt2.y = cvRound(y0 - 2000 * ( a)) + offset.y;


        // Draw on cv_im_out
        cv::line(image, pt1, pt2, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        if (--max_lines == 0) {
            std::cout << "Too many lines: " << std::to_string(lines.size()) << " lines to draw.\n";
            break;
        }
    }

}

void drawClusteredPoints(cv::Mat& image, const DBSCAN& clustering, const cv::Point2i& offset) {

    std::vector<cv::Scalar> colors{getNColors(clustering.getMaxClusterID() + 1)};
    const std::vector<Pointdb>& points{clustering.getPoints()};

    for (unsigned long idx{0}; idx < points.size(); idx++) {
        cv::Point2i center = cv::Point2i{static_cast<int>(points[idx].x), static_cast<int>(points[idx].y)} + offset;
        cv::circle(image, center,
                   1, colors.at(points[idx].clusterID + 1), 2);
    }
}


void drawCycle(cv::Mat& image, const std::vector<cv::Point2f>& points, const cv::Point2f& offset,
               const cv::Scalar& color, bool arrowed) {
    if (points.size() <= 1) return;
    if (arrowed) {
        for (unsigned long i{0}; i < points.size() - 1; ++i) {
            cv::arrowedLine(image, points.at(i) + offset, points.at(i+1) + offset, color);
        }
        cv::arrowedLine(image, points.back() + offset, points.front() + offset, color);
    }
    else {
        for (unsigned long i{0}; i < points.size() - 1; ++i) {
            cv::line(image, points.at(i) + offset, points.at(i+1) + offset, color);
        }
        cv::line(image, points.back() + offset, points.front() + offset, color);
    }

}


void drawPoints(cv::Mat &image_draw, std::vector<cv::Point2f> &points,
                const cv::Point2f &offset, const cv::Scalar& color)
{
    if (!points.empty()) {
        for (const cv::Point2f& intersect : points) {
            cv::circle(image_draw, intersect + offset, 1, color, 3);
        }
    }

}

void drawPoints(cv::Mat &image_draw, std::vector<cv::Point2i> &points,
                const cv::Point2i &offset, const cv::Scalar& color)
{
    if (!points.empty()) {
        for (const cv::Point2i& intersect : points) {
            cv::circle(image_draw, intersect + offset, 1, color, 3);
        }
    }

}


