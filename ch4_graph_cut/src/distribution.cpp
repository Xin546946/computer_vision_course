#include "distribution.h"
#include "display.h"
#include <opencv2/core/core.hpp>
Distribution::Distribution(cv::Mat img, std::vector<cv::Point> foreground,
                           std::vector<cv::Point> background)
    : gmms_{GMM(img, foreground, 2), GMM(img, background, 2)},
      lamda_(100),
      img_(img.clone()) {
    for (int i = 0; i < gmms_.size(); i++) {
        gmms_[i].run(20);
    }
}

// todo change the name : flag
cv::Mat Distribution::get_probability_map(int flag) {
    cv::Mat log_prob = cv::Mat::zeros(img_.size(), CV_64FC1);
    cv::log(gmms_[1 - flag].get_prob(), log_prob);
    return -lamda_ * log_prob;
}
