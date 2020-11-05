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
<<<<<<< HEAD
=======
    // laplacian_gvf_x_(cv::Mat::zeros(grad_x_original_.size(), CV_32F)),
    // laplacian_gvf_y_(cv::Mat::zeros(grad_y_original_.size(), CV_32F)) {
>>>>>>> 60ff58ecd7aef526f2a39818e4c093130b5cd31c
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

    cv::Mat laplacian_gvf_x;  // Laplacian of gvf_x_
    cv::Mat laplacian_gvf_y;  // Laplacian of gvf_y_
    cv::Laplacian(gvf_x_, laplacian_gvf_x, CV_32F, 1, cv::BORDER_REFLECT);
    cv::Laplacian(gvf_y_, laplacian_gvf_y, CV_32F, 1, cv::BORDER_REFLECT);

    cv::Mat data_term_dev_x;
    cv::multiply(mag_grad_original_, gvf_x_ - grad_x_original_,
                 data_term_dev_x);

    cv::Mat data_term_dev_y;
    cv::multiply(mag_grad_original_, gvf_y_ - grad_y_original_,
                 data_term_dev_y);

    gvf_x_ += 1e-8 * (param_gvf_.mu_ * laplacian_gvf_x - data_term_dev_x);
    gvf_y_ += 1e-8 * (param_gvf_.mu_ * laplacian_gvf_y - data_term_dev_y);

    cv::Scalar color(0, 255, 0);
    cv::Mat tmp = cv::Mat::zeros(gvf_y_.size(), CV_8UC3);
    cv::Mat tmp_gvf_x;
    cv::Mat tmp_gvf_y;

    cv::normalize(gvf_x_, tmp_gvf_x, -1, 1, cv::NORM_MINMAX);
    cv::normalize(gvf_y_, tmp_gvf_y, -1, 1, cv::NORM_MINMAX);
    draw_optical_flow(tmp_gvf_x, tmp_gvf_y, tmp, 8, 10, color);
    disp_image(tmp, "gvf", 1);
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