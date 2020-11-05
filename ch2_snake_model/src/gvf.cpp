#include "gvf.h"
#include "display.h"
#include <cmath>
#include <iostream>
#include <opencv2/imgproc.hpp>
ParamGVF::ParamGVF(float mu, float sigma, float init_step_size)
    : smooth_term_weight_(mu), sigma_(sigma), init_step_size_(init_step_size_) {
}

/**
 * @brief Construct a new GVF::GVF object
 *
 * @param grad_x_original
 * @param grad_y_original
 * @param param_gvf
 */
GVF::GVF(cv::Mat grad_x_original, cv::Mat grad_y_original,
         const ParamGVF& param_gvf)
    : GradientDescentBase(param_gvf.init_step_size_),
      param_gvf_(param_gvf),
      mag_grad_original_(cv::Mat::zeros(grad_x_original.size(), CV_32F)),
      grad_x_original_(grad_x_original.clone()),
      grad_y_original_(grad_y_original.clone()),
      gvf_x_(grad_x_original.clone()),
      gvf_y_(grad_y_original.clone()),
      data_term_dev_x_(cv::Mat::zeros(grad_x_original.size(), CV_32F)),
      data_term_dev_y_(cv::Mat::zeros(grad_x_original.size(), CV_32F)),
      laplacian_gvf_x_(cv::Mat::zeros(grad_x_original.size(), CV_32F)),
      laplacian_gvf_y_(cv::Mat::zeros(grad_x_original.size(), CV_32F)) {
    cv::Mat grad_x_2, grad_y_2;
    cv::multiply(grad_x_original_, grad_x_original_, grad_x_2);
    cv::multiply(grad_y_original_, grad_y_original_, grad_y_2);
    mag_grad_original_ = grad_x_2 + grad_y_2;
}

void GVF::initialize() {
    gvf_x_ = grad_x_original_.clone();
    gvf_y_ = grad_y_original_.clone();
}

void GVF::update() {
    cv::GaussianBlur(gvf_x_, gvf_x_, cv::Size(3, 3), param_gvf_.sigma_,
                     param_gvf_.sigma_);
    cv::GaussianBlur(gvf_y_, gvf_y_, cv::Size(3, 3), param_gvf_.sigma_,
                     param_gvf_.sigma_);

    cv::Laplacian(gvf_x_, laplacian_gvf_x_, CV_32F, 1, cv::BORDER_REFLECT);
    cv::Laplacian(gvf_y_, laplacian_gvf_y_, CV_32F, 1, cv::BORDER_REFLECT);

    // cv::Mat data_term_dev_x;
    cv::multiply(mag_grad_original_, gvf_x_ - grad_x_original_,
                 data_term_dev_x_);

    // cv::Mat data_term_dev_y;
    cv::multiply(mag_grad_original_, gvf_y_ - grad_y_original_,
                 data_term_dev_y_);

    gvf_x_ += step_size_ * (param_gvf_.smooth_term_weight_ * laplacian_gvf_x_ -
                            data_term_dev_x_);
    gvf_y_ += step_size_ * (param_gvf_.smooth_term_weight_ * laplacian_gvf_y_ -
                            data_term_dev_y_);

    display_gvf(gvf_x_, gvf_y_);
}

float GVF::compute_energy() {
    cv::Mat data_term_x, data_term_y;
    cv::multiply(data_term_dev_x_, gvf_x_ - grad_x_original_, data_term_x);
    cv::multiply(data_term_dev_y_, gvf_y_ - grad_y_original_, data_term_y);
    cv::Mat smooth_term, data_term;
    cv::Mat gvf_x_dev_x, gvf_x_dev_y, gvf_y_dev_x, gvf_y_dev_y;
    cv::Sobel(gvf_x_, gvf_x_dev_x, CV_32F, 1, 0, 3);
    cv::Sobel(gvf_x_, gvf_x_dev_y, CV_32F, 0, 1, 3);
    cv::Sobel(gvf_y_, gvf_y_dev_x, CV_32F, 1, 0, 3);
    cv::Sobel(gvf_y_, gvf_y_dev_y, CV_32F, 0, 1, 3);
    cv::Mat gvf_x_dev_x_2, gvf_x_dev_y_2, gvf_y_dev_x_2, gvf_y_dev_y_2;
    cv::pow(gvf_x_dev_x, 2.0f, gvf_x_dev_x_2);
    cv::pow(gvf_x_dev_y, 2.0f, gvf_x_dev_y_2);
    cv::pow(gvf_y_dev_x, 2.0f, gvf_y_dev_x_2);
    cv::pow(gvf_y_dev_y, 2.0f, gvf_y_dev_y_2);
    smooth_term = gvf_x_dev_x_2 + gvf_x_dev_y_2 + gvf_y_dev_x_2 + gvf_y_dev_y_2;

    return cv::sum(smooth_term + data_term)[0];
}

void GVF::roll_back_state() {
    gvf_x_ = last_gvf_x_;
    gvf_y_ = last_gvf_y_;
}

void GVF::back_up_state() {
    last_gvf_x_ = gvf_x_;
    last_gvf_y_ = gvf_y_;
}

std::vector<cv::Mat> GVF::get_result_gvf() const {
    std::vector<cv::Mat> gvf_result(2);
    gvf_result[0] = gvf_x_.clone();
    gvf_result[1] = gvf_y_.clone();
    return gvf_result;
}

void GVF::print_terminate_info() const {
    std::cout << "GVF iteration finished." << std::endl;
}