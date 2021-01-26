#pragma once
#include <opencv2/core/core.hpp>
#include <vector>
struct LineParam {
    LineParam() = default;
    LineParam(double a, double b) : a_(a), b_(b) {
    }

    double a_;
    double b_;
};

LineParam line_fitting(const std::vector<cv::Point2d>& points);