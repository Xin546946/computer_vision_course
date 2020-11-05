#include "gvf.h"
#include "display.h"
#include <cmath>
#include <iostream>
#include <opencv2/imgproc.hpp>
ParamGVF::ParamGVF(float mu, float sigma, float step)
    : mu_(mu), sigma_(sigma), step_(step) {
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
    : param_gvf_(param_gvf),
      mag_grad_original_(cv::Mat::zeros(grad_x_original.size(), CV_32F)),
      grad_x_original_(grad_x_original.clone()),
      grad_y_original_(grad_y_original.clone()),
      gvf_x_(grad_x_original.clone()),
      gvf_y_(grad_y_original.clone()) {
    // laplacian_gvf_x_(cv::Mat::zeros(grad_x_original_.size(), CV_32F)),
    // laplacian_gvf_y_(cv::Mat::zeros(grad_y_original_.size(), CV_32F)) {
    cv::Mat grad_x_2, grad_y_2;
    cv::multiply(grad_x_original_, grad_x_original_, grad_x_2);
    cv::multiply(grad_y_original_, grad_y_original_, grad_y_2);
    mag_grad_original_ = grad_x_2 + grad_y_2;
}

void GVF::initialize() {
    // grad_x_original_.copyTo(gvf_x_);
    // grad_y_original_.copyTo(gvf_y_);
    gvf_x_ = grad_x_original_.clone();
    gvf_y_ = grad_y_original_.clone();
}

void GVF::update() {
    cv::GaussianBlur(gvf_x_, gvf_x_, cv::Size(3, 3), 3, 3);
    cv::GaussianBlur(gvf_y_, gvf_y_, cv::Size(3, 3), 3, 3);
    cv::Mat laplacian_gvf_x_, laplacian_gvf_y_;
    cv::Laplacian(gvf_x_, laplacian_gvf_x_, CV_32F, 1, cv::BORDER_REFLECT);
    cv::Laplacian(gvf_y_, laplacian_gvf_y_, CV_32F, 1, cv::BORDER_REFLECT);

    cv::Mat data_term_dev_x;
    cv::multiply(mag_grad_original_, gvf_x_ - grad_x_original_,
                 data_term_dev_x);
    cv::Mat data_term_dev_y;
    cv::multiply(mag_grad_original_, gvf_y_ - grad_y_original_,
                 data_term_dev_y);

    gvf_x_ += param_gvf_.step_ *
              (param_gvf_.mu_ * laplacian_gvf_x_ - data_term_dev_x);
    gvf_y_ += param_gvf_.step_ *
              (param_gvf_.mu_ * laplacian_gvf_y_ - data_term_dev_y);
    /*
        cv::Mat energy_x, energy_y, energy_data;
        cv::multiply(gvf_x_ - grad_x_original_, gvf_x_ - grad_x_original_,
                     energy_x);
        cv::multiply(gvf_y_ - grad_y_original_, gvf_y_ - grad_y_original_,
                     energy_y);
        cv::multiply(energy_x, energy_y, energy_data);
        float energy;*/
    cv::Mat gvf_x_normal, gvf_y_normal;
    cv::normalize(gvf_x_, gvf_x_normal, -1, 1, cv::NORM_MINMAX);
    cv::normalize(gvf_y_, gvf_y_normal, -1, 1, cv::NORM_MINMAX);
    cv::Mat gvf_show = cv::Mat::zeros(gvf_x_.size(), gvf_x_.type());
    cv::cvtColor(gvf_show, gvf_show, CV_GRAY2RGB);
    // cv::normalize(gvf_show, gvf_show);
    cv::Scalar color(0, 255, 0);
    draw_optical_flow(gvf_x_normal, gvf_y_normal, gvf_show, 5, 10, color);
    disp_image(gvf_show, "gvf", 0.1);
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