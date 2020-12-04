#include "gmm.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/*--------------------------------------------------------
#####################implementation: ConfigParam #####################
---------------------------------------------------------*/
gmm::ConfigParam::ConfigParam(double a, double alpha, double fore_threshold)
    : a_(a), alpha_(alpha), fore_threshold_(fore_threshold) {
}

/*--------------------------------------------------------
#####################implementation: GaussianParam #####################
---------------------------------------------------------*/

/*--------------------------------------------------------
#####################implementation: GMM #####################
---------------------------------------------------------*/
gmm::GMM::GMM(int num_gaussian, ConfigParam config_param)
    : num_gaussian_(num_gaussian), config_param_(config_param), model_param_() {
}

void gmm::GMM::add_sample(double sample) {
    if (!is_in_gmm(sample)) {
        replace_model();
    } else {
        update_gmm();
    }
}

bool gmm::GMM::is_in_gmm(double sample) {
}