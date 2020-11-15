#include "level_set_cv.h"
#include "display.h"
#include "level_set_utils.h"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <opencv2/core.hpp>

ParamLevelSetCV ::ParamLevelSetCV(double forground_weight,
                                  double background_weight, double eps,
                                  double step_size, double length_term_weight,
                                  double gradient_term_weight)
    : forground_weight_(forground_weight),
      background_weight_(background_weight),
      eps_(eps),
      step_size_(step_size),
      length_term_weight_(length_term_weight),
      gradient_term_weight_(gradient_term_weight) {
}

LevelSetCV::LevelSetCV(cv::Mat image, const ParamLevelSetCV& param)
    : GradientDescentBase(param.step_size_),
      level_set_(image.rows, image.cols,
                 cv::Point(image.cols / 2, image.rows / 2),
                 std::min(image.rows, image.cols) / 2.5f),
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

/**
 * @brief update the lvl set according to the direction of gradient descent
 *
 */
void LevelSetCV::update_level_set() {
    cv::Mat update_step_data_term =
        -param_.step_size_ * compute_derivative_data_term(
                                 level_set_, image_64f_,
                                 param_.forground_weight_,
                                 param_.background_weight_, center_foreground_,
                                 center_background_, param_.eps_);
    cv::Mat update_step_length_term =
        param_.step_size_ * param_.length_term_weight_ *
        compute_derivative_length_term(level_set_, param_.eps_);

    cv::Mat update_step_gradient_term =
        param_.step_size_ * param_.gradient_term_weight_ *
        compute_derivative_gradient_term(level_set_);

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

    cv::Mat update_step = update_step_data_term + update_step_length_term +
                          update_step_gradient_term;
    level_set_.add(update_step);
}

void LevelSetCV::roll_back_state() {
    level_set_ = last_level_set_;
}
void LevelSetCV::back_up_state() {
    last_level_set_ = level_set_;
}
void LevelSetCV::print_terminate_info() const {
    std::cout << "Level set iteration finished." << std::endl;
}
double LevelSetCV::compute_energy() const {
    return 0;
}
void LevelSetCV::initialize() {
    // initialize lvl set :   sdf already initilized in constructor

    // initilize centers :
}

std::string LevelSetCV::return_drive_class_name() const {
    return "Level Set CV Model";
}
void LevelSetCV::update_center() {
    center_foreground_ =
        compute_center(image_64f_, level_set_, param_.eps_, false);
    std::cout << "center_foreground " << center_foreground_ << '\n';
    center_background_ =
        compute_center(image_64f_, level_set_, param_.eps_, true);
    std::cout << "center_background " << center_background_ << '\n';
}
void LevelSetCV::update() {
    update_center();
    update_level_set();

    cv::Mat vis_sdf_draw = draw_sdf_map(level_set_);
    cv::Mat vis_sdf_with_contour =
        draw_points(vis_sdf_draw, level_set_.get_contour_points(),
                    cv::Scalar(255, 255, 255));

    cv::Mat vis_seg_image = image_3_channel.clone();
    vis_seg_image = draw_points(vis_seg_image, level_set_.get_contour_points(),
                                cv::Scalar(0, 0, 255));

    cv::Mat vis_label = level_set_.get_fore_background_label_map();
    vis_label.convertTo(vis_label, CV_8UC1);
    cv::cvtColor(vis_label, vis_label, CV_GRAY2BGR);

    cv::Mat vis;
    cv::hconcat(vis_sdf_with_contour, vis_seg_image, vis);
    cv::hconcat(vis, vis_label, vis);

    cv::imshow("left: level set, mid: seg on original image, right : label ",
               vis);
    cv::waitKey(1);
}