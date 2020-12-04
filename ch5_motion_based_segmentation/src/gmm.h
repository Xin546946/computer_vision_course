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
    double mean_ = 0;
    double var_ = 50;
    double weight_ = 0;
};

double compute_gaussian_pdf(GaussianParam param, double sample);

bool operator<(const gmm::GaussianParam& lhs, const gmm::GaussianParam& rhs);

struct ModelParam {
    ModelParam(int num_gaussian);
    std::vector<GaussianParam> param_;
    void sort();
};
std::ostream& operator<<(std::ostream& os, const gmm::ModelParam& model_param);

enum class GMMStatus { NOT_INILIALIZED = 1, INITIALIZED = 2 };

class GMM {
   public:
    GMM(int num_gaussian, ConfigParam config_param);
    void add_sample(double sample);
    ModelParam get_model_param() const;
    bool is_last_sample_foreground() const;

   private:
    bool is_in_gmm(double sample);
    void replace_model(double sample);
    void update_gmm(double sample);
    int num_gaussians_;
    ConfigParam config_param_;  // todo make this static
    ModelParam model_param_;
    GMMStatus status_ = gmm::GMMStatus::NOT_INILIALIZED;
};
}  // namespace gmm
