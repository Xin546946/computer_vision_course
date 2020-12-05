/**
______________________________________________________________________
*********************************************************************
* @brief  This file is developed for the course of ShenLan XueYuan:
* Fundamental implementations of Computer Vision
* all rights preserved
* @author Xin Jin, Zhaoran Wu
* @contact: xinjin1109@gmail.com, zhaoran.wu1@gmail.com
*
______________________________________________________________________
*********************************************************************
**/
#include "gmm.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <random>
#include <vector>
void test_fit_model(std::vector<double>& means, std::vector<double>& vars, std::vector<double>& weights, double a = 1.5,
                    double alpha = 0.7, double fore_threshold = 0.25);

std::vector<double> generate_gmm_data(int num_want, const std::vector<double>& means, const std::vector<double>& vars,
                                      const std::vector<double>& weights);
int main(int argc, char** argv) {
    // test0
    std::vector<double> means{3, 8, 10};
    std::vector<double> vars{4, 3, 4};
    std::vector<double> weights{0.1, 0.5, 0.4};
    test_fit_model(means, vars, weights);

    return 0;
}

/*--------------------------------------------------------
#####################implementation: test function #####################
---------------------------------------------------------*/

void test_fit_model(std::vector<double>& means, std::vector<double>& vars, std::vector<double>& weights, double a,
                    double alpha, double fore_threshold) {
    // generate gmm samples
    int num_want = 1000;
    std::vector<double> samples = generate_gmm_data(num_want, means, vars, weights);
    // declare gmm

    gmm::ConfigParam config_param(a, alpha, fore_threshold);
    gmm::GMM gmm;

    for (const auto& sample : samples) {
        gmm.add_sample(sample);
    }
    gmm::ModelParam model_param = gmm.get_model_param();
    std::cout << model_param << std::endl;
}

std::vector<double> generate_gmm_data(int num_want, const std::vector<double>& means, const std::vector<double>& vars,
                                      const std::vector<double>& weights) {
    assert(means.size() == vars.size() && means.size() == weights.size());
    int sz = means.size();
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::vector<std::normal_distribution<double>> dists;
    for (int i = 0; i < sz; i++) {
        dists.emplace_back(means[i], vars[i]);
    }

    std::vector<double> result;

    for (int n = 0; n < num_want; n++) {
        double tmp = 0.0;
        for (int i = 0; i < sz; i++) {
            tmp += weights[i] * dists[i](gen);
        }
        result.push_back(tmp);
    }

    return result;
}