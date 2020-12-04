#include "gmm.h"
#include <algorithm>
#include <cmath>
/*--------------------------------------------------------
#####################implementation: ConfigParam #####################
---------------------------------------------------------*/
gmm::ConfigParam::ConfigParam(double a, double alpha, double fore_threshold)
    : a_(a), alpha_(alpha), fore_threshold_(fore_threshold) {
}

/*--------------------------------------------------------
#####################implementation: GaussianParam #####################
---------------------------------------------------------*/
gmm::GaussianParam::GaussianParam(double mean, double var, double weight) : mean_(mean), var_(var), weight_(weight) {
}

/*--------------------------------------------------------
#####################implementation: GaussianModel #####################
---------------------------------------------------------*/
gmm::GaussianModel::GaussianModel(GaussianParam param) : param_(param) {
}

double gmm::GaussianModel::compute_gaussian_pdf(double sample) const {
    return (1 / (sqrt(2 * M_PI) * param_.var_)) * exp(-0.5 * pow((sample - param_.mean_) / param_.var_, 2));
}

/*--------------------------------------------------------
#####################implementation: ModelParam #####################
---------------------------------------------------------*/
bool operator<(const gmm::GaussianParam& lhs, const gmm::GaussianParam& rhs) {
    return (lhs.weight_ / lhs.var_) < (rhs.weight_ / rhs.var_);
}

void gmm::ModelParam::sort() {
    std::sort(this->param_.begin(), this->param_.end());
}

/*--------------------------------------------------------
#####################implementation: GMM #####################
---------------------------------------------------------*/
gmm::GMM::GMM(int num_gaussians, ConfigParam config_param)
    : num_gaussians_(num_gaussians), config_param_(config_param), model_param_() {
}

void gmm::GMM::add_sample(double sample) {
    if (!is_in_gmm(sample)) {
        replace_model(sample);
    } else {
        update_gmm();
    }
}

bool gmm::GMM::is_in_gmm(double sample) {
    return abs(sample - model_param_.param_.top().mean_) < config_param_.a_ * model_param_.param_.top().var_;
}

void gmm::GMM::replace_model(double sample) {
    model_param_.sort();
    model_param_.param_[0].mean_ = sample;
    model_param_.param_[0].var_ = 50;
}

void gmm::GMM::update_gmm() {
    update_weight();
    double ro = compute_ro();
    update_mean();
    update_var();
}

void gmm::GMM::update_weight(double sample) {
    for (auto it = model_param_.param_.begin(); it != model_param_.param_.end(); it++) {
        *it.weight_ = (1 - config_param_.alpha_) * it * weight_ + config_param_.alpha_*
    }
}
