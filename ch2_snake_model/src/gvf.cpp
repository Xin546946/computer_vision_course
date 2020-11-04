#include "gvf.h"
#include <opencv2/imgproc.hpp>
/**
 * @brief Construct a new GVF::GVF object
 *
 * @param img
 * @param param_gvf
 */
GVF::GVF(cv::Mat2f grad_original, cv::Mat laplacian_original,
         const ParamGVF& param_gvf)
    : param_gvf_(param_gvf),
      mag_grad_original_(cv::Mat::zeros(
          cv::Size(grad_original_.rows, grad_original_.cols), CV_32F)),
      grad_original_(grad_original.clone()),
      laplacian_original_(laplacian_original.clone()) {
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
    cv::Mat laplacian_gvf_ = laplacian_original_.clone();
    cv::multiply(mag_grad_original_, (gvf_ - grad_original_),
                 data_term_derivative);
    gvf_ += data_term_derivative - param_gvf_.mu_ * laplacian_gvf_;
    cv::Laplacian(gvf_, laplacian_gvf_, CV_32F, 1, cv::BORDER_REFLECT);
}