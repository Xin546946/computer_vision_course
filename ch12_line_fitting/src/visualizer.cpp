#include "visualizer.h"
#include "opencv_utils.h"
#include <opencv2/imgproc.hpp>

inline double compute_x(double a, double b, double y) {
    return (y - b) / (a + 1e-5);
}

inline double compute_y(double a, double b, double x) {
    return a * x + b;
}

Visualizer::Visualizer(int height, int width) : board_(height, width, CV_8UC3, cv::Scalar(255, 255, 255)) {
}

void Visualizer::add_point(const cv::Point2d& point, bool is_inlier) {
    cv::Scalar color = (is_inlier) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::circle(board_, point, 1, color, 2, cv::LINE_AA);
}

void Visualizer::add_line(double a, double b, bool draw_thresh_line = false, double threshold = 0.0) {
    int x1, x2, y1, y2;

    if (a < 1 && a > -1) {
        x1 = -1;
        y1 = compute_y(a, b, x1);

        x2 = board_.cols;
        y2 = compute_y(a, b, x2);
    } else {
        y1 = -1;
        x1 = compute_x(a, b, y1);

        y2 = board_.rows;
        x2 = compute_x(a, b, y2);
    }
    cv::line(board_, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 0, 0), 2, cv::LINE_AA);

    if (draw_thresh_line) {
        cv::Scalar color(50, 50, 50);

        draw_dashed_line(board_, cv::Point(x1, y1 + threshold), cv::Point(x2, y2 + threshold), color, 2, cv::LINE_AA);
        draw_dashed_line(board_, cv::Point(x1, y1 - threshold), cv::Point(x2, y2 - threshold), color, 2, cv::LINE_AA);
    }
}
void Visualizer::show(int delay) const {
    cv::imshow("result", board_);
    cv::waitKey(delay);
}
