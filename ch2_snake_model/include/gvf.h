#pragma once
#include "gradient_descent_base.h"
#include <opencv2/core.hpp>

struct ParamGVF {
    ParamGVF(float mu, int max_iteration, float sigma);
    ParamGVF(const ParamGVF& param_gvf);

    float mu_;
    int max_iterations_;
    float sigma_;
};

class GVF : public GradientDescentBase {
   public:
    // laplacian_original: Ixx**2 + Iyy**2
    GVF(cv::Mat grad_x_original, cv::Mat grad_y_original,
        const ParamGVF& param_gvf = ParamGVF(0.2f, 50, 1));

   private:
    void initialize() override;
    void update() override;

    ParamGVF param_gvf_;
    cv::Mat mag_grad_original_;  // 1 channel grad_x**2+grad_y**2
    cv::Mat grad_original_;      // 2 channel grad_original: grad_x, grad_y
    cv::Mat gvf_;                // 2 channels c1 dx, c2 dy
};
