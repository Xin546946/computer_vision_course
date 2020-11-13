#include "sdf_map.h"
#include <iostream>
#include <opencv2/imgproc.hpp>

inline bool is_contour_x_dire(cv::Mat im, int r, int c) {
    assert(!im.empty() && r < im.rows && r >= 0 && c < im.cols && c >= 0);
    return ((im.at<double>(r, c - 1) * im.at<double>(r, c + 1)) < 0);
}

inline bool is_contour_y_dire(cv::Mat im, int r, int c) {
    assert(!im.empty() && r < im.rows && r >= 0 && c < im.cols && c >= 0);
    return ((im.at<double>(r - 1, c) * im.at<double>(r + 1, c)) < 0);
}

SDFMap::SDFMap(int rows, int cols, cv::Point2d center, double radius)
    : map_(cv::Mat::zeros(cv::Size(cols, rows), CV_64F)) {
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            map_.at<double>(r, c) =
                std::sqrt(pow(r - center.y, 2) + pow(c - center.x, 2)) - radius;
        }
    }
}

/**
 * @brief get the contour from level set 0;
 *
 * @return cv::Mat N*2 Mat, each row contain a Point2d(x,y)
 */
cv::Mat SDFMap::draw_contour(cv::Mat img) const {
    cv::Mat result = img.clone();
    if (result.channels() == 1) {
        cv::cvtColor(result, result, CV_GRAY2BGR);
    }

    if (result.type() != CV_8UC3) {
        result.convertTo(result, CV_8UC3);
    }

    for (int r = 1; r < result.rows - 1; r++) {
        for (int c = 1; c < result.cols - 1; c++) {
            if (is_contour_x_dire(map_, r, c) ||
                is_contour_y_dire(map_, r, c)) {
                result.at<cv::Vec3b>(r, c) = cv::Vec3b(255, 255, 255);
            }
        }
    }
    return result;
}
cv::Mat SDFMap::get_fore_background_label_map() const {
    cv::Mat fore_background = map_.clone();
    cv::threshold(map_, fore_background, 0, 255, cv::THRESH_BINARY_INV);
    return fore_background;
}

double SDFMap::get_gradient_magnitude_level_set() {
    cv::Mat map_dev_x;
    cv::Sobel(map_, map_dev_x, CV_64F, 1, 0, 3);
    cv::Mat map_dev_y;
    cv::Sobel(map_, map_dev_y, CV_64F, 0, 1, 3);
    cv::Mat mag_grad_map;
    cv::sqrt(map_dev_x.mul(map_dev_x) + map_dev_y.mul(map_dev_y), mag_grad_map);
    return cv::sum(0.5 * (mag_grad_map - 1.0).mul(mag_grad_map - 1.0))[0];
}