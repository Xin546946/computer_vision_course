#include "gmm.h"
#include <algorithm>
#include <cmath>
#include <iostream>
namespace gmm {

/*--------------------------------------------------------
#####################implementation: ConfigParam #####################
---------------------------------------------------------*/
ConfigParam::ConfigParam(double a, double alpha, double fore_threshold)
    : a_(a), alpha_(alpha), fore_threshold_(fore_threshold) {
}

/*--------------------------------------------------------
#####################implementation: GaussianParam #####################
---------------------------------------------------------*/
GaussianParam::GaussianParam(double mean, double var, double weight) : mean_(mean), var_(var), weight_(weight) {
}

/*--------------------------------------------------------
#####################implementation: compute_gaussian_pdf #####################
---------------------------------------------------------*/

double compute_gaussian_pdf(gmm::GaussianParam param, double sample) {
    return (1 / (sqrt(2 * M_PI) * param.var_)) * exp(-0.5 * pow((sample - param.mean_) / param.var_, 2));
}

/*--------------------------------------------------------
#####################implementation: ModelParam #####################
---------------------------------------------------------*/
bool operator<(const GaussianParam& lhs, const GaussianParam& rhs) {
    return (lhs.weight_ / lhs.var_) < (rhs.weight_ / rhs.var_);
}

void ModelParam::sort() {
    std::sort(this->param_.begin(), this->param_.end());
}

std::ostream& operator<<(std::ostream& os, const ModelParam& model_param) {
    for (int i = 0; i < model_param.param_.size(); i++) {
        const gmm::GaussianParam& param = model_param.param_[i];
        std::cout << "model id : " << i << " mean : " << param.mean_ << " var : " << param.var_
                  << " weight : " << param.weight_ << '\n';
    }
}

/*--------------------------------------------------------
#####################implementation: GMM #####################
---------------------------------------------------------*/
GMM::GMM(int num_gaussians, ConfigParam config_param) : num_gaussians_(num_gaussians), config_param_(config_param) {
}

ModelParam GMM::get_model_param() const {
    return this->model_param_;
}
void GMM::add_sample(double sample) {
    if (!is_in_gmm(sample)) {
        replace_model(sample);
    } else {
        update_gmm(sample);
    }
}

bool GMM::is_in_gmm(double sample) {
    model_param_.sort();
    return abs(sample - model_param_.param_[0].mean_) < config_param_.a_ * model_param_.param_[0].var_;
}

void GMM::replace_model(double sample) {
    model_param_.sort();
    model_param_.param_[0].mean_ = sample;
    model_param_.param_[0].var_ = 50;
}

void GMM::update_gmm(double sample) {
    for (int i = 0; i < num_gaussians_; i++) {
        double sample_pdf = gmm::compute_gaussian_pdf(model_param_.param_[i], sample);
        model_param_.param_[i].weight_ =
            (1 - config_param_.alpha_) * model_param_.param_[i].weight_ + config_param_.alpha_ * sample_pdf;
        double ro = config_param_.alpha_ * sample_pdf;
        model_param_.param_[i].mean_ = (1 - config_param_.alpha_) * model_param_.param_[i].mean_ + ro * sample;
        model_param_.param_[i].var_ = (1 - config_param_.alpha_) * model_param_.param_[i].var_ +
                                      ro * pow(sample - model_param_.param_[i].mean_, 2);
    }
}

}  // namespace gmm