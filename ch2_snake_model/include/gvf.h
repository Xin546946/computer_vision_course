#pragma once
#include "gradient_descent_base.h"
#include <opencv2/core.hpp>

struct ParamGVF {
    ParamGVF(const float mu, const float max_iteration, float sigma)
        : mu_(mu), max_iterations_(max_iteration), sigma_(sigma){};
    float mu_;
    int max_iterations_;
    float sigma_;
};

class GVF : public GradientDescentBase {
   public:
    GVF(cv::Mat img, ParamGVF param_gvf) : param_gvf_(param_gvf(0.2f, 50, 1));

   private:
    void initialize() override;
    void update() override;
    ParamGVF param_gvf_;
};
