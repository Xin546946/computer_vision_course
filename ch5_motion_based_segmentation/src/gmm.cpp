#include "gmm.h"
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
gmm::GaussianParam::GaussianParam(double mean, double var, double weight)
    : mean_(mean), var_(var), weight_(weight) {
}

/*--------------------------------------------------------
#####################implementation: ModelParam #####################
---------------------------------------------------------*/

int gmm::ModelParam::get_size() const {
    return param_.size();
}

/*--------------------------------------------------------
#####################implementation: GMM #####################
---------------------------------------------------------*/
gmm::GMM::GMM(int num_gaussians, ConfigParam config_param)
    : num_gaussians_(num_gaussians),
      config_param_(config_param),
      model_param_() {
}

void gmm::GMM::add_sample(double sample) {
    if (!is_in_gmm(sample)) {
        replace_model(sample);
    } else {
        update_gmm();
    }
}

bool gmm::GMM::is_in_gmm(double sample) {
    return abs(sample - model_param_.param_.top().mean_) <
           config_param_.a_ * model_param_.param_.top().var_;
}

void gmm::GMM::replace_model(double sample) {
    double weight = model_param_.param_.top().weight_;
    model_param_.param_.pop();
    model_param_.param_.emplace(sample, 50, weight);
}

void gmm::GMM::update_gmm() {
    update_weight();
    double ro = compute_ro();
    update_mean();
    update_var();
}

void gmm::GMM::update_weight(double sample) {
    for (auto it = model_param_.param_.begin(); it != model_param_.param_.end();
         it++) {
        *it.weight
    }
}
