/**
______________________________________________________________________
*********************************************************************
* @brief  This file is developed for the course of ShenLan XueYuan:
* Fundamental implementations of Computer Vision
* all rights preserved
* @author Xin Jin, Zhaoran Wu
* @contact: xinjin1109@gmail.com, zhaoran.wu1@gmail.com
*
______________________________________________________________________
*********************************************************************
**/

#include "sdf_map.h"
#include "display.h"
#include "level_set_utils.h"
#include <iostream>
#include <opencv2/imgproc.hpp>

/**
 * @brief draw sdf map for visualization
 *
 * @param sdf_map to be visulized visualization
 * @return cv::Mat the visualzation image
 */
cv::Mat draw_sdf_map(const SDFMap& sdf_map) {
    assert(!sdf_map.map_.empty());
    return apply_jetmap(sdf_map.map_);
}

inline bool is_contour_x_dire(cv::Mat im, int r, int c) {
    assert(!im.empty() && r < im.rows && r >= 0 && c < im.cols && c >= 0);
    return ((im.at<double>(r, c - 1) * im.at<double>(r, c + 1)) < 0);
}

inline bool is_contour_y_dire(cv::Mat im, int r, int c) {
    assert(!im.empty() && r < im.rows && r >= 0 && c < im.cols && c >= 0);
    return ((im.at<double>(r - 1, c) * im.at<double>(r + 1, c)) < 0);
}

SDFMap::SDFMap(int rows, int cols, cv::Point center, double radius)
    : map_(cv::Mat::zeros(cv::Size(cols, rows), CV_64F)) {
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            map_.at<double>(r, c) =
                std::sqrt(pow(r - center.y, 2) + pow(c - center.x, 2)) - radius;
        }
    }
}
SDFMap::SDFMap(int rows, int cols)
    : map_(2 * cv::Mat::ones(cv::Size(cols, rows), CV_64F)) {
    double percentage = 0.2;
    map_(cv::Rect2d(
        cv::Point(round(percentage * cols), round(percentage * rows)),
        cv::Point(cols - round(percentage * cols),
                  rows - round(percentage * rows)))) =
        -2 * cv::Mat::ones(cv::Size(cols - round(percentage * cols) * 2,
                                    rows - round(percentage * rows) * 2),
                           CV_64F);
}

cv::Mat SDFMap::get_fore_background_label_map() const {
    cv::Mat fore_background = map_.clone();
    cv::threshold(map_, fore_background, 0, 255, cv::THRESH_BINARY_INV);
    return fore_background;
}

double SDFMap::get_gradient_magnitude_level_set() const {
    cv::Mat map_dev_x = do_sobel(map_, 0);
    cv::Mat map_dev_y = do_sobel(map_, 1);
    cv::Mat mag_grad_map;
    cv::sqrt(map_dev_x.mul(map_dev_x) + map_dev_y.mul(map_dev_y), mag_grad_map);
    return cv::sum(0.5 * (mag_grad_map - 1.0).mul(mag_grad_map - 1.0))[0];
}

void SDFMap::add(cv::Mat step) {
    map_ += step;
}

/**
 * @brief return a N*2 mat, each row is a point2d(x,y);
 *
 * @return cv::Mat
 */
cv::Mat SDFMap::get_contour_points() const {
    std::vector<cv::Vec2d> contour_vec;

    for (int r = 1; r < map_.rows - 1; r++) {
        for (int c = 1; c < map_.cols - 1; c++) {
            if (is_contour_x_dire(map_, r, c) ||
                is_contour_y_dire(map_, r, c)) {
                contour_vec.emplace_back(c, r);
            }
        }
    }

    cv::Mat contour(cv::Size(2, contour_vec.size()), CV_64FC1);
    for (int i = 0; i < contour_vec.size(); i++) {
        contour.at<cv::Vec2d>(i) = contour_vec[i];
    }

    return contour;
}