#include "distribution.h"
#include <opencv2/core/core.hpp>
Distribution::Distribution(cv::Mat img, std::vector<cv::Point> foreground,
                           std::vector<cv::Point> background)
    : gmms_{GMM(img, foreground, 2), GMM(img, background, 2)}, lamda_(1.0) {
}

double Distribution::compute_weight(int row, int col, int flag) const {
    return -lamda_ * log(gmms_[flag].)
}