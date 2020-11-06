#pragma once
#include "gradient_descent_base.h"
#include <opencv2/core.hpp>
#include <vector>

struct ParamGVF {
    /**
     * @brief Construct a new Param G V F object
     *
     * @param smooth_term_weight
     * @param sigma
     * @param init_step_size
     */
    ParamGVF(float smooth_term_weight = 10, float sigma = 3.0f,
             float init_step_size = 1e-7);
    float init_step_size_;
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

    cv::Mat last_gvf_x_;
    cv::Mat last_gvf_y_;

    cv::Mat laplacian_gvf_x_;  // Laplacian of gvf_x_
    cv::Mat laplacian_gvf_y_;  // Laplacian of gvf_y_
};
