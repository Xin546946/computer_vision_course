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
    double get_weight(int row, int col, int flag) const;

   private:
    void fit_distribution(
        cv::Mat img, std::vector<cv::Point> foreground,
        std::vector<cv::Point> background);  // todo fit the model using gmm
    cv::Mat foreground_probability_map_;     // todo they need to be initialized
    cv::Mat background_probability_map_;     // todo
    std::array<std::array<double, 3>, 2> miu_;
    std::array<std::array<double, 3>, 2> sigma_;
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