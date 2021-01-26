#include "visualizer.h"
#include "opencv_utils.h"
#include <opencv2/imgproc.hpp>

inline double compute_x(double a, double b, double y) {
    return (y - b) / (a + 1e-5);
}

inline double compute_y(double a, double b, double x) {
    return a * x + b;
}

Visualizer::Visualizer(double x_min, double x_max, double y_min, double y_max) : up_left_(x_min, y_min) {
    int height = std::ceil(y_max - y_min);
    int width = std::ceil(x_max - x_min);
    board_ = cv::Mat(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    int row_x_axis = -up_left_[1];
    int col_y_axis = -up_left_[0];

    cv::arrowedLine(board_, cv::Point(0, row_x_axis), cv::Point(board_.cols - 1, row_x_axis), cv::Scalar(0, 255, 255),
                    2, cv::LINE_AA);

    cv::arrowedLine(board_, cv::Point(col_y_axis, 0), cv::Point(col_y_axis, board_.rows - 1), cv::Scalar(0, 255, 255),
                    2, cv::LINE_AA);
}

void Visualizer::add_point(double x, double y, bool is_inlier) {
    cv::Scalar color = (is_inlier) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::circle(board_, cv::Point(std::round(x - up_left_[0]), std::round(y - up_left_[1])), 1, color, 2, cv::LINE_AA);
}

void Visualizer::add_line(double a, double b, bool draw_thresh_line, double threshold) {
    int x1, x2, y1, y2;

    if (a < 1 && a > -1) {
        x1 = up_left_[0];
        y1 = compute_y(a, b, x1);

        x2 = board_.cols - up_left_[0];
        y2 = compute_y(a, b, x2);
    } else {
        y1 = up_left_[0];
        x1 = compute_x(a, b, y1);

        y2 = board_.rows - up_left_[0];
        x2 = compute_x(a, b, y2);
    }

    x1 -= up_left_[0];
    x2 -= up_left_[0];
    y1 -= up_left_[1];
    y2 -= up_left_[1];

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
