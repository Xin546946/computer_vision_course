#include "distribution.h"
#include <opencv2/core/core.hpp>
Distribution::Distribution(cv::Mat img, std::vector<cv::Point> foreground,
                           std::vector<cv::Point> background)
    : foreground_probability_map_(cv::Mat::zeros(img.size(), img.type())),
      background_probability_map_(cv::Mat::zeros(img.size(), img.type())) {
    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
        }
    }
}