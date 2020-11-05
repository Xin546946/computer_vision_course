#pragma once
#include "gradient_descent_base.h"
#include <opencv2/core.hpp>
#include <vector>
struct ParamGVF {
    ParamGVF(float smooth_term_weight = 0.2f, float sigma = 3.0f,
             float init_step_size = 1e-12);
    float step_;
    float smooth_term_weight_;
    float sigma_;
};

class GVF : public GradientDescentBase {
   public:
    GVF(cv::Mat grad_x_original, cv::Mat grad_y_original,
        const ParamGVF& param_gvf = ParamGVF(0.2f, 1.0f));
    std::vector<cv::Mat> get_result_gvf() const;

   private:
    void initialize() override;
    void update() override;

    float compute_energy() override;
    void roll_back_state() override;
    void back_up_state() override;

    void print_terminate_info() const override;

    ParamGVF param_gvf_;

    cv::Mat mag_grad_original_;  // grad_x_original_**2 + grad_y_original_**2
    cv::Mat grad_x_original_;    // partial derivative w.r.t. x
    cv::Mat grad_y_original_;    // partial derivative w.r.t. y

    cv::Mat gvf_x_;
    cv::Mat gvf_y_;
};
