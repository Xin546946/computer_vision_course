#include "sdf_map.h"
#include "display.h"  //todo just for test
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

SDFMap::SDFMap(int rows, int cols, cv::Point2d center, double radius)
    : map_(cv::Mat::zeros(cv::Size(cols, rows), CV_64F)) {
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            map_.at<double>(r, c) =
                std::sqrt(pow(r - center.y, 2) + pow(c - center.x, 2)) - radius;
        }
    }
}

cv::Mat SDFMap::get_fore_background_label_map() const {
    cv::Mat fore_background = map_.clone();
    cv::threshold(map_, fore_background, 0, 255, cv::THRESH_BINARY_INV);
    return fore_background;
}

double SDFMap::get_gradient_magnitude_level_set() {
    cv::Mat map_dev_x;
    cv::Sobel(map_, map_dev_x, CV_64F, 1, 0, 3);
    disp_image(map_dev_x, "mag dev x", 0);
    cv::Mat map_dev_y;
    cv::Sobel(map_, map_dev_y, CV_64F, 0, 1, 3);
    disp_image(map_dev_y, "mag dev y", 0);
    cv::Mat mag_grad_map;
    cv::sqrt(map_dev_x.mul(map_dev_x) + map_dev_y.mul(map_dev_y), mag_grad_map);
    disp_image(mag_grad_map, "mag mag", 0);
    return cv::sum(0.5 * (mag_grad_map - 1.0).mul(mag_grad_map - 1.0))[0];
}