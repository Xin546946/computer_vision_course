#pragma once

#include <iostream>
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
    double mean_ = -100;
    double var_ = 1e-5;
    double weight_ = 0;
};

double compute_gaussian_pdf(GaussianParam param, double sample);

bool operator<(const gmm::GaussianParam& lhs, const gmm::GaussianParam& rhs);

struct ModelParam {
    ModelParam(int num_gaussian);
    std::vector<GaussianParam> param_;
    void sort();
    void normalize_weight();
};
std::ostream& operator<<(std::ostream& os, const gmm::ModelParam& model_param);

class GMM {
   public:
    GMM(int num_gaussian, ConfigParam config_param);
    void add_sample(double sample);
    ModelParam get_model_param() const;
    bool is_last_sample_foreground() const;

   private:
    int get_gm_id(double sample);
    void replace_model(double sample);
    void update_gmm(double sample, int id);
    int num_gaussians_;
    ConfigParam config_param_;  // todo make this static
    ModelParam model_param_;
};
}  // namespace gmm
