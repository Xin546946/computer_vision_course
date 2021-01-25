#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * @brief Line Param : rho(pixel) and theta(degree)
 *
 */
struct LineParam {
    LineParam(double rho, double theta) : rho_(rho), theta_(theta) {
    }
    double rho_ = -1;
    double theta_ = -1;
};

/**
 * @brief Line detection using hough transformation
 *
 * @param img
 * @return std::vector<LineParam>
 */
std::vector<LineParam> line_detection(cv::Mat img);

/**
 * @brief draw line with line params on the img
 *
 * @param img
 * @param line_param
 */
void draw_line(cv::Mat img, const std::vector<LineParam>& line_param);