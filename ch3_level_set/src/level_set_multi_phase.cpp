#include "level_set_multi_phase.h"
#include "display.h"
#include "level_set_cv.h"
#include "level_set_utils.h"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <opencv2/core.hpp>

ParamLevelSetMP ::ParamLevelSetMP(double weight_1, double weight_2,
                                  double weight_3, double weight_4, double eps,
                                  double step_size, double length_term_weight,
                                  double gradient_term_weight)
    : weight_1_(weight_1),
      weight_2_(weight_2),
      weight_3_(weight_3),
      weight_4_(weight_4),
      eps_(eps),
      step_size_(step_size),
      length_term_weight_(length_term_weight),
      gradient_term_weight_(gradient_term_weight) {
}

LevelSetMP::LevelSetMP(cv::Mat image, const ParamLevelSetMP& param)
    : GradientDescentBase(param.step_size_),
      level_set_1_(round(image.rows / 2), image.cols),
      level_set_2_(image.rows, round(image.cols / 2)),
      /*                  cv::Point(image.cols / 2, image.rows / 2),
                       std::min(image.rows, image.cols) / 2.5f) ,*/
      last_level_set_1_(level_set_1_),
      last_level_set_2_(level_set_2_),
      param_(param),
      image_3_channel(image.clone()),
      image_64f_(image.size(), CV_64FC1),
      center_1_(0.0),
      center_2_(80.0),
      center_3_(160.0),
      center_4_(255.0),
      last_center_1_(0.0),
      last_center_2_(80.0),
      last_center_3_(160.0),
      last_center_4_(255.0) {
    image.convertTo(image_64f_, CV_64FC1);
}

cv::Mat compute_derivative_data_term_mp1(HightMap level_set_1_,
                                         HightMap level_set_2_,
                                         cv::Mat image_64f_,
                                         ParamLevelSetMP param_,
                                         double center_1_, double center_2_) {
    cv::Mat e1 = compute_square_diff(
        image_64f_,
        center_1_ * cv::Mat::ones(image_64f_.size(), image_64f_.type()));
    cv::Mat e2 = compute_square_diff(
        image_64f_,
        center_2_ * cv::Mat::ones(image_64f_.size(), image_64f_.type()));

    cv::Mat M1 = dirac(level_set_1_).mul(heaviside(level_set_2_)) +
                 dirac(level_set_2_).mul(heaviside(level_set_1_));
    cv::Mat M2 = dirac(level_set_2_).mul(1 - heaviside(level_set_1_)) -
                 dirac(level_set_1_).mul(heaviside(level_set_2_));

    cv::Mat update_step_data_term_mp1 = e1.mul(M1) + e2.mul(M2);
    return update_step_data_term_mp1;
}

cv::Mat compute_derivative_data_term_mp2(HightMap level_set_1_,
                                         HightMap level_set_2_,
                                         cv::Mat image_64f_,
                                         ParamLevelSetMP param_,

                                         double center_3_, double center_4_) {
    cv::Mat e3 = compute_square_diff(
        image_64f_,
        center_3_ * cv::Mat::ones(image_64f_.size(), image_64f_.type()));
    cv::Mat e4 = compute_square_diff(
        image_64f_,
        center_4_ * cv::Mat::ones(image_64f_.size(), image_64f_.type()));

    cv::Mat M3 = dirac(level_set_1_).mul(1 - heaviside(level_set_2_)) -
                 dirac(level_set_2_).mul(heaviside(level_set_1_));
    cv::Mat M4 = -dirac(level_set_1_).mul(1 - heaviside(level_set_2_)) -
                 dirac(level_set_2_).mul(1 - heaviside(level_set_1_));

    cv::Mat update_step_data_term_mp2 = e3.mul(M3) + e4.mul(M4);
    return update_step_data_term_mp2;
}
void LevelSetMP::roll_back_state() {
    level_set_1_ = last_level_set_1_;
    level_set_2_ = last_level_set_2_;
}
void LevelSetMP::back_up_state() {
    last_level_set_1_ = level_set_1_;
    last_level_set_2_ = level_set_2_;
}
void LevelSetMP::print_terminate_info() const {
    std::cout << "Level set iteration finished." << std::endl;
}

// todo
double LevelSetMP::compute_energy() const {
    return 0;
}
void LevelSetMP::initialize() {
    // initialize lvl set :   sdf already initilized in constructor

    // initilize centers :
}

std::string LevelSetMP::return_drive_class_name() const {
    return "Level Set Multi Phase Model";
}
void LevelSetMP::update_center() {
    center_1_ = compute_center(image_64f_, level_set_1_, param_.eps_, false);
    // std::cout << "center_foreground " << center_foreground_1_ << '\n';
    center_2_ = compute_center(image_64f_, level_set_1_, param_.eps_, true);
    // std::cout << "center_background " << center_background_1_ << '\n';
    center_3_ = compute_center(image_64f_, level_set_2_, param_.eps_, false);
    // std::cout << "center_foreground " << center_foreground_2_ << '\n';
    center_4_ = compute_center(image_64f_, level_set_2_, param_.eps_, true);
    // std::cout << "center_background " << center_background_2_ << '\n';
}

void LevelSetMP::update_level_set_mp() {
    cv::Mat update_step_length_term_1 =
        param_.step_size_ * param_.length_term_weight_ *
        compute_derivative_length_term(level_set_1_, param_.eps_);

    cv::Mat update_step_gradient_term_1 =
        param_.step_size_ * param_.gradient_term_weight_ *
        compute_derivative_gradient_term(level_set_1_);
    cv::Mat update_step_length_term_2 =
        param_.step_size_ * param_.length_term_weight_ *
        compute_derivative_length_term(level_set_2_, param_.eps_);

    cv::Mat update_step_gradient_term_2 =
        param_.step_size_ * param_.gradient_term_weight_ *
        compute_derivative_gradient_term(level_set_2_);

    cv::Mat update_step_data_term_1 = compute_derivative_data_term_mp1(
        level_set_1_, level_set_2_, image_64f_, param_, center_1_, center_2_);
    cv::Mat update_step_data_term_2 = compute_derivative_data_term_mp2(
        level_set_1_, level_set_2_, image_64f_, param_, center_3_, center_4_);

    cv::Mat update_step_1 = update_step_data_term_1 +
                            update_step_length_term_1 +
                            update_step_gradient_term_1;
    cv::Mat update_step_2 = update_step_data_term_2 +
                            update_step_length_term_2 +
                            update_step_gradient_term_2;
    level_set_1_.add(update_step_1);
    level_set_2_.add(update_step_2);
}
void LevelSetMP::update() {
    update_center();
    update_level_set_mp();

    cv::Mat vis_sdf_draw = draw_sdf_map(level_set_1_);
    cv::Mat vis_sdf_with_contour =
        draw_points(vis_sdf_draw, level_set_1_.get_contour_points(),
                    cv::Scalar(255, 255, 255));

    cv::Mat vis_seg_image = image_3_channel.clone();
    vis_seg_image =
        draw_points(vis_seg_image, level_set_1_.get_contour_points(),
                    cv::Scalar(0, 0, 255));

    cv::Mat vis_label = level_set_1_.get_fore_background_label_map();
    vis_label.convertTo(vis_label, CV_8UC1);
    cv::cvtColor(vis_label, vis_label, CV_GRAY2BGR);

    cv::Mat vis;
    cv::hconcat(vis_sdf_with_contour, vis_seg_image, vis);
    cv::hconcat(vis, vis_label, vis);

    cv::imshow("left: level set, mid: seg on original image, right : label ",
               vis);
    cv::waitKey(1);
}
