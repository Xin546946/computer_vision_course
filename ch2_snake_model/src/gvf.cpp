#include "gvf.h"
/**
 * @brief Construct a new GVF::GVF object
 *
 * @param img
 * @param param_gvf
 */
GVF::GVF(cv::Mat2f grad_img_original, const ParamGVF& param_gvf)
    : param_gvf_(param_gvf),
      mag_original_(cv::Mat::zeros(
          cv::Size(mag_original_.rows, mag_original_.cols), CV_32F)),
      grad_img_original_(grad_img_original.clone()) {
    for (int r = 0; r < grad_img_original.rows; r++) {
        for (int c = 0; c < grad_img_original.cols; c++) {
            mag_original_.at<float>(r, c) =
                cv::norm(grad_img_original_.at<cv::Vec2f>(r, c), cv::NORM_L2);
        }
    }
}

void GVF::initialize() {
    grad_img_gvf_ = grad_img_original_.clone();
}

void GVF::update() {
}