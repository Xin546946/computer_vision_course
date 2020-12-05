#include "gmm.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
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
ModelParam::ModelParam(int num_gaussian) : param_(num_gaussian) {
}
bool cmp_priority(const GaussianParam& lhs, const GaussianParam& rhs) {
    return (lhs.weight_ / lhs.var_) < (rhs.weight_ / rhs.var_);
}

bool cmp_weight(const GaussianParam& lhs, const GaussianParam& rhs) {
    return lhs.weight_ > rhs.weight_;
}

void ModelParam::sort_with_priority() {
    std::sort(this->param_.begin(), this->param_.end(), cmp_priority);
}
void ModelParam::sort_with_weight() {
    std::sort(this->param_.begin(), this->param_.end(), cmp_weight);
}

void ModelParam::normalize_weight() {
    double sum_weight = 0.0;

    for (int i = 0; i < param_.size(); i++) {
        sum_weight += param_[i].weight_;
    }

    std::for_each(param_.begin(), param_.end(), [=](GaussianParam& param) { param.weight_ /= sum_weight; });
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
GMM::GMM(int num_gaussians, ConfigParam config_param)
    : num_gaussians_(num_gaussians), config_param_(config_param), model_param_(num_gaussians) {
}

ModelParam GMM::get_model_param() const {
    return this->model_param_;
}

void GMM::add_sample(double sample) {
    model_param_.sort_with_priority();
    int id = get_gm_id(sample);
    if (id == -1) {
        // std::cout << "Current sample " << sample << "is not in the gaussian model" << '\n';
        replace_model(sample);
    } else {
        // std::cout << "Current sample " << sample << "is in the " << id << "-th gaussian model" << '\n';
        update_gmm(sample, id);
    }
}
bool GMM::is_in_model(double sample, int id_model) const {
    return abs(sample - model_param_.param_[id_model].mean_) < config_param_.a_ * model_param_.param_[id_model].var_;
}
int GMM::get_gm_id(double sample) {
    for (int id = 0; id < num_gaussians_; id++) {
        bool in_model = is_in_model(sample, id);
        if (in_model) {
            return id;
        }
    }
    return -1;
}
void GMM::replace_model(double sample) {
    model_param_.param_[0].mean_ = sample;
    model_param_.param_[0].var_ = 50;
    model_param_.param_[0].weight_ = 0.1;
    model_param_.normalize_weight();
}

void GMM::update_gmm(double sample, int id) {
    model_param_.param_[id].weight_ =
        (1 - config_param_.alpha_) * model_param_.param_[id].weight_ + config_param_.alpha_;
    double ro = config_param_.alpha_ / model_param_.param_[id].weight_;
    model_param_.param_[id].mean_ = (1 - ro) * model_param_.param_[id].mean_ + ro * sample;
    model_param_.param_[id].var_ = std::sqrt((1 - ro) * std::pow(model_param_.param_[id].var_, 2.0) +
                                             ro * std::pow(sample - model_param_.param_[id].mean_, 2.0));
    model_param_.normalize_weight();
}

bool GMM::is_in_foreground(double sample) {
    model_param_.sort_with_weight();
    double sum_weight = 0.0;
    double id = 0;
    while (sum_weight < config_param_.fore_threshold_) {
        sum_weight += model_param_.param_[id].weight_;
        id++;
    }

    for (int i = 0; i <= id; i++) {
        if (is_in_model(sample, i)) {
            return false;
        }
    }

    return true;
}

}  // namespace gmm