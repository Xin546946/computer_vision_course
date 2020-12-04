#pragma once

#include <queue>
#include <vector>

namespace gmm {

struct ConfigParam {
    ConfigParam(double a, double alpha, double fore_threshold);
    double a_;
    double alpha_;
    double fore_threshold_;
};

struct GaussianParam {
    GaussianParam(double mean, double var, double weight);
    double mean_ = 0;
    double var_ = 50;
    double weight_ = 1;
};

struct Gaussian {
    Gaussian(GaussianParam param);
    void compute_gaussian_pdf() const;
};

bool operator<(const GaussianParam& lhs, const GaussianParam& rhs) {
    return (lhs.weight_ / lhs.var_) > (rhs.weight_ / rhs.var_);
}
struct ModelParam {
    std::priority_queue<GaussianParam> param_;

    int get_size() const;
};

class GMM {
   public:
    GMM(int num_gaussian, ConfigParam config_param);
    void add_sample(double sample);
    ModelParam get_model_param() const;
    bool is_foreground() const;

   private:
    bool is_in_gmm(double sample);
    void replace_model(double sample);
    void update_gmm();
    void update_mean();
    void update_var();
    void update_weight(double sample);
    double compute_ro();
    int num_gaussians_;
    ConfigParam config_param_;
    ModelParam model_param_;
};
}  // namespace gmm
