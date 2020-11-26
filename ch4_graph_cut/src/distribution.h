#pragma once
#include "gmm.h"
#include <array>
#include <opencv2/core/core.hpp>
#include <vector>
double compute_weight(const cv::Vec3f& color1, const cv::Vec3f& color2);

class Distribution {
   public:
    Distribution(cv::Mat img, std::vector<cv::Point> foreground,
                 std::vector<cv::Point> background);
    // 0 is foreground, 1 is background
    cv::Mat get_probability_map(int flag) const;

   private:
    double compute_weight(int row, int col, int flag) const;

    std::array<GMM, 2> gmms_;
    double lamda_;
};

/*--------------------------------------------------------
#####################implementation: inline #####################
---------------------------------------------------------*/
// todo
// inline double compute_weight(const cv::Vec3f& color1, const cv::Vec3f&
// color2,
//                             double sigma) {
//    return std::exp(1 / (2 * sigma * sigma) *
//                    std::pow(cv::norm(color1 - color2, cv::NORM_L2), 2));
//}