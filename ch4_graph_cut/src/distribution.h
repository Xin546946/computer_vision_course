
#pragma once
#include "gmm.h"
#include <array>
#include <opencv2/core/core.hpp>
#include <vector>

class Distribution {
   public:
    Distribution(cv::Mat img, std::vector<cv::Point> foreground,
                 std::vector<cv::Point> background);
    // 0 is foreground, 1 is background
    cv::Mat get_probability_map(int id);

   private:
    std::array<GMM, 2> gmms_;
    double lamda_;
    cv::Mat img_;
};

/*--------------------------------------------------------
#####################implementation: inline #####################
---------------------------------------------------------*/

inline double compute_weight(const cv::Vec3f& color1, const cv::Vec3f& color2,
                             double sigma_square_inv = 2.5e-5) {
    cv::Vec3f diff = color2 - color1;
    return 20 * exp(-0.5 * sigma_square_inv * (diff.dot(diff)));
}