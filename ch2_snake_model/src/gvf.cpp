#include "gvf.h"
#include <cmath>
#include <opencv2/imgproc.hpp>
/**
 * @brief Construct a new GVF::GVF object
 *
 * @param img
 * @param param_gvf
 */
GVF::GVF(cv::Mat grad_original, const ParamGVF& param_gvf)
    : param_gvf_(param_gvf),
      mag_grad_original_(cv::Mat::zeros(
          cv::Size(grad_original_.rows, grad_original_.cols), CV_32F)),
      grad_original_(grad_original.clone()) {
    for (int r = 0; r < grad_original.rows; r++) {
        for (int c = 0; c < grad_original.cols; c++) {
            mag_grad_original_.at<float>(r, c) =
                cv::norm(grad_original_.at<cv::Vec2f>(r, c), cv::NORM_L2);
        }
    }
}

void GVF::initialize() {
    gvf_ = grad_original_.clone();
}

void GVF::update() {
    cv::Mat data_term_derivative;
    // TODO
    // gvf_(i) = gvf_(i) +  param_gvf_.mu * laplacian_gvf_(i) -
    // mag_grad_original_ * (gvf_(i) - grad_original_(i))
    for (int channel = 0; channel < 2; channel++) {
        gvf_[channel] = gvf_[channel]
    }
}