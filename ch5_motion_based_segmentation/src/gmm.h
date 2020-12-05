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
    double mean_ = -100.0;
    double var_ = 1e-5;
    double weight_ = 0.0;
};

double compute_gaussian_pdf(GaussianParam param, double sample);

bool operator<(const gmm::GaussianParam& lhs, const gmm::GaussianParam& rhs);

struct ModelParam {
    ModelParam(int num_gaussian);
    std::vector<GaussianParam> param_;
    void sort_with_priority();  // bigger at end
    void sort_with_weight();    // bigger at front
    void normalize_weight();
};
std::ostream& operator<<(std::ostream& os, const gmm::ModelParam& model_param);

class GMM {
   public:
    GMM();
    void add_sample(double sample);
    ModelParam get_model_param() const;
    bool is_in_foreground(double sample);

    static int num_gaussians_;
    static ConfigParam config_param_;

   private:
    bool is_in_model(double sample, int id_model) const;
    int get_gm_id(double sample);
    void replace_model(double sample);
    void update_gmm(double sample, int id);

    ModelParam model_param_;
};

}  // namespace gmm
