#pragma once

#include <opencv2/core/core.hpp>
#include <queue>
#include <vector>

struct GaussianParam {
    double mean_;
    double var_;
    double weight_;
};

namespace gmm {

struct ConfigParam {
    ConfigParam(double a, double alpha, double fore_threshold);
    double a_;
    double alpha_;
    double fore_threshold_;
};

bool operator<(const GaussianParam& lhs, const GaussianParam& rhs) {
    return (lhs.weight_ / lhs.var_) < (rhs.weight_ / rhs.var_);
}
struct ModelParam {
    std::priority_queue<GaussianParam> model_param;
};

class GMM {
   public:
    GMM(int num_gaussian, ConfigParam config_param);
    void add_sample(double sample);
    ModelParam get_model_param() const;
    bool is_foreground() const;

   private:
    bool is_in_gmm(double sample);
    void replace_model();
    void update_gmm();

    int num_gaussian_;
    ConfigParam config_param_;
    ModelParam model_param_;
};
}  // namespace gmm
