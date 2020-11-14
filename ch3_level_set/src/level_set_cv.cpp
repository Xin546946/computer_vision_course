#include "level_set_cv.h"
#include "level_set_utils.h"
#include "opencv2/highgui.hpp"
#include <opencv2/core.hpp>
LevelSetCV::LevelSetCV(cv::Mat image, const ParamLevelSetCV& param)
    : GradientDescentBase(param_.step_size_),
      level_set_(image.rows, image.cols,
                 cv::Point(image.cols / 2, image.rows / 2),
                 std::min(image.rows, image.cols) / 2.5f),
      param_(param),
      image_(image.clone()),
      center_background_(255.0),
      center_foreground_(0.0) {
}

/**
 * @brief update the lvl set according to the direction of gradient descent
 *
 */
void LevelSetCV::update_level_set() {
    cv::Mat update_step_data_term;
    update_step_data_term =
        param_.step_size_ * compute_derivative_data_term(
                                level_set_, image_, param_.forground_weight_,
                                param_.background_weight_, center_foreground_,
                                center_background_, param_.eps_);
    cv::Mat update_step_length_term;
    cv::Mat update_step_gradient_term;

    cv::Mat vis;
    cv::hconcat(update_step_data_term, update_step_length_term, vis);
    cv::hconcat(vis, update_step_gradient_term, vis);
    cv::imshow("top: data term, mid : lenght_term, down : gradient_term", vis);

    cv::Mat update_step = update_step_data_term + update_step_length_term +
                          update_step_gradient_term;
    level_set_.add(update_step);
}