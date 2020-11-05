#pragma once
#include "gradient_descent_base.h"
#include <opencv2/core.hpp>
#include <vector>
struct ParamGVF {
    ParamGVF(float mu = 0.2f, float sigma = 1.0f, float step = 1e-12);
    // ParamGVF(const ParamGVF& param_gvf) = default;
    float step_;
    float mu_;
    float sigma_;
};

class GVF : public GradientDescentBase {
   public:
    // grad_x_original: gaussian(img).dx
    // grad_y_original: gaussian(img).dy
    GVF(cv::Mat grad_x_original, cv::Mat grad_y_original,
        const ParamGVF& param_gvf = ParamGVF(0.2f, 1.0f));
    std::vector<cv::Mat> get_result_gvf() const;

   private:
    void initialize() override;
    void update() override;
    void print_terminate_info() const override;

    ParamGVF param_gvf_;
    cv::Mat mag_grad_original_;  // grad_x_original_**2 + grad_y_original_**2
    cv::Mat grad_x_original_;    // partial derivative w.r.t. x
    cv::Mat grad_y_original_;    // partial derivative w.r.t. y
    cv::Mat gvf_x_;
    cv::Mat gvf_y_;
};
