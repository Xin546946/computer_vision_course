/**
______________________________________________________________________
*********************************************************************
* @brief This file is developed for the course of ShenLan XueYuan:
* Fundamental implementations of Computer Vision
* all rights preserved
* @author Xin Jin, Zhaoran Wu
* @contact: xinjin1109@gmail.com, zhaoran.wu1@gmail.com
*
______________________________________________________________________
*********************************************************************
**/
#include "ransac.h"
#include "math_utils.h"
#include <iostream>
#include <vector>

/**
 * @brief Pick two points randomly for given points
 *
 * @param points
 * @param num
 * @return std::pair<cv::Point2d, cv::Point2d>
 */
std::pair<cv::Point2d, cv::Point2d> pick_points(std::vector<cv::Point2d> points, int num) {
    std::vector<int> points_id;

    int id = generate_random_data(1, 0, points.size() - 1)[0];
    points_id.push_back(id);

    while (1) {
        id = generate_random_data(1, 0, points.size() - 1)[0];
        if (id != points_id.back()) {
            points_id.push_back(id);
            break;
        }
    }

    return {points[points_id[0]], points[points_id[1]]};
}

LineParam fit_line(cv::Point2d point_front, cv::Point2d point_back) {
    double a = (point_back.y - point_front.y) / (point_back.x - point_front.x + 1e-6);
    double b = point_front.y - a * point_front.x;
    LineParam line_param(a, b);
    return line_param;
}

/**
 * @brief compute the number of outliers for given line and points with threshold
 *
 * @param line_param
 * @param datas
 * @param threshold
 * @return int
 */
int compute_num_outliers(LineParam line_param, const std::vector<cv::Point2d>& datas, double threshold) {
    int num_outlier = 0;
    double den_line = std::sqrt(line_param.a_ * line_param.a_ + 1.0) + 1e-6;
    for (cv::Point2d data : datas) {
        double dist = std::abs(line_param.a_ * data.x + line_param.b_ - data.y) / den_line;
        if (dist > threshold) {
            num_outlier++;
        }
    }
    return num_outlier;
}

LineParam line_fitting(const std::vector<cv::Point2d>& points) {
    // todo implement line_fitting function
}
