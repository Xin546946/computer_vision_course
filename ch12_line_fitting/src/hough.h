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
 * @brief
 *
 * @param [in] img
 * @param [in] theta_resolution
 * @param [in] rho_resoulution
 * @return std::vector<LineParam>
 */
std::vector<LineParam> line_detection(cv::Mat img, double theta_resolution = 1, double rho_resoulution = 1);

/**
 * @brief draw line with line params on the img
 *
 * @param img
 * @param line_param
 */
void draw_line(cv::Mat img, const std::vector<LineParam>& line_param);