#include "level_set_lbf.h"
#include "display.h"
#include "level_set_cv.h"
#include "level_set_utils.h"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <opencv2/core.hpp>

ParamLevelSetLBF::ParamLevelSetLBF(double forground_weight,
                                   double background_weight, double eps,
                                   double step_size, double length_term_weight,
                                   double gradient_term_weight, int window_size)
    : ParamLevelSet(forground_weight, background_weight, eps, step_size,
                    length_term_weight, gradient_term_weight),
      window_size_(window_size) {
}

LevelSetLBF::LevelSetLBF(cv::Mat image, const ParamLevelSetLBF& param)
    : GradientDescentBase(param.step_size_),
      level_set_(image.rows, image.cols),
      /*                  cv::Point(image.cols / 2, image.rows / 2),
                       std::min(image.rows, image.cols) / 2.5f) ,*/
      last_level_set_(level_set_),
      param_(param),
      image_3_channel(image.clone()),
      image_64f_(image.size(), CV_64FC1),
      center_background_(0.0),
      center_foreground_(255.0),
      last_center_background_(0.0),
      last_center_foreground_(255.0) {
    image.convertTo(image_64f_, CV_64FC1);
}

void LevelSetLBF::roll_back_state() {
    level_set_ = last_level_set_;
}
void LevelSetLBF::back_up_state() {
    last_level_set_ = level_set_;
}
void LevelSetLBF::print_terminate_info() const {
    std::cout << "Level set iteration finished." << std::endl;
}

// todo for slding window
double LevelSetLBF::compute_energy() const {
    double data_term_energy = compute_data_term_energy(
        level_set_, image_64f_, param_.forground_weight_,
        param_.background_weight_, center_foreground_, center_background_,
        param_.eps_);
    double length_term_energy =
        compute_length_term_energy(level_set_, param_.eps_);
    double gradient_preserve_energy =
        compute_gradient_preserve_energy(level_set_);
    std::cout << "||"
              << "data term energy: " << data_term_energy << " || ";
    std::cout << "length term energy: " << length_term_energy << " || ";
    std::cout << "gradient preserve energy: " << gradient_preserve_energy
              << " || " << std::endl;
    return data_term_energy + param_.length_term_weight_ * length_term_energy +
           param_.gradient_term_weight_ * gradient_preserve_energy;
}
void LevelSetLBF::initialize() {
    // initialize lvl set :   sdf already initilized in constructor

    // initilize centers :
}

std::string LevelSetLBF::return_drive_class_name() const {
    return "Level Set LBF Model";
}

void LevelSetLBF::update_center() {
    center_foreground_ =
        compute_center(image_64f_, level_set_, param_.eps_, false);
    std::cout << "center_foreground " << center_foreground_ << '\n';
    center_background_ =
        compute_center(image_64f_, level_set_, param_.eps_, true);
    std::cout << "center_background " << center_background_ << '\n';
}

// todo compute_data_term_derivitive_in_window(r,c)
// todo  update_center_in_window(r, c);
void LevelSetLBF::update() {
    cv::Mat total_data_term_derivative =
        cv::Mat::zeros(image_64f_.size(), image_64f_.type());
    for (int r = 0; r < image_64f_.rows; r++) {
        for (int c = 0; c < image_64f_.cols; c++) {
            update_center_in_window(r, c);
            total_data_term_derivative +=
                compute_data_term_derivative_in_window(r, c);
        }
    }
    cv::Mat update_step_length_term =
        param_.step_size_ * param_.length_term_weight_ *
        compute_derivative_length_term(level_set_, param_.eps_);

    cv::Mat update_step_gradient_term =
        param_.step_size_ * param_.gradient_term_weight_ *
        compute_derivative_gradient_term(level_set_);
    cv::Mat update_step_data_term =
        param_.step_size_ * total_data_term_derivative;
    cv::Mat update_step = update_step_data_term + update_step_length_term +
                          update_step_gradient_term;
    level_set_.add(update_step);

    cv::Mat vis;
    cv::Mat vis_update_data_term = get_float_mat_vis_img(update_step_data_term);
    cv::Mat vis_update_lenght_term =
        get_float_mat_vis_img(update_step_length_term);
    cv::Mat vis_update_graient_term =
        get_float_mat_vis_img(update_step_gradient_term);

    cv::hconcat(update_step_data_term, update_step_length_term, vis);
    cv::hconcat(vis, update_step_gradient_term, vis);

    cv::imshow("top: data term, mid : lenght_term, down : gradient_term", vis);
    cv::waitKey(1);
}

void LevelSetLBF::update_center_in_window(int row, int col) {
}

cv::Mat LevelSetLBF::compute_data_term_derivative_in_window(int row,
                                                            int col) const {
}