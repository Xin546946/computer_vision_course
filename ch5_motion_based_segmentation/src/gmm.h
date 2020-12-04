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
    GaussianParam() = default;
    GaussianParam(double mean, double var, double weight);
    double mean_ = 0;
    double var_ = 50;
    double weight_ = 1;
};

struct GaussianModel {
    GaussianModel(GaussianParam param);
    double compute_gaussian_pdf(double sample) const;
    GaussianParam param_;
};

struct ModelParam {
    std::vector<GaussianParam> param_;
    void sort();
};

class GMM {
   public:
    GMM(int num_gaussian, ConfigParam config_param);
    void add_sample(double sample);
    ModelParam get_model_param() const;
    bool is_last_sample_foreground() const;

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
    GaussianModel model_;
};
}  // namespace gmm
