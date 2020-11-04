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
    GVF(cv::Mat2f grad_img_original,
        const ParamGVF& param_gvf = ParamGVF(0.2f, 50, 1));

   private:
    void initialize() override;
    void update() override;

    ParamGVF param_gvf_;
    cv::Mat mag_original_;
    cv::Mat2f grad_img_original_;
    cv::Mat2f grad_img_gvf_;
};
