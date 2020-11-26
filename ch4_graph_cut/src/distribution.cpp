#include "distribution.h"
#include <opencv2/core/core.hpp>
Distribution::Distribution(cv::Mat img, std::vector<cv::Point> foreground,
                           std::vector<cv::Point> background)
    : gmms_{GMM(img, foreground, 2), GMM(img, background, 2)},
      lamda_(1.0),
      img_(img) {
}

cv::Mat Distribution::get_probability_map(int flag) {
    cv::Mat log_prob = cv::Mat::zeros(img_.size(), CV_64FC1);
    cv::log(gmms_[flag].get_prob(), log_prob);
    return -lamda_ * log_prob;
}
