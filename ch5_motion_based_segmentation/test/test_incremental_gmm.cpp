#include "gmm.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <random>
#include <vector>
void test_fit_model(std::vector<double>& means, std::vector<cv::Matx33d>& vars,
                    std::vector<double>& weights, double a = 1.5,
                    double alpha = 0.7, double fore_threshold = 0.25);
std::vector<double> generate_gmm_data(std::vector<double>& means,
                                      std::vector<cv::Matx33d>& vars,
                                      std::vector<double>& weights);
int main(int argc, char** argv) {
    std::vector<double> means{10, 5, 6};
    std::vector<cv::Matx33d> vars{
        4 * cv::Matx33d::eye(), 3 * cv::Matx33d::eye(), 4 * cv::Matx33d::eye()};
    std::vector<double> weights{0.3, 0.5, 0.2};

    generate_gmm_data(means, vars, weights);
    // test0
    /*     std::vector<double> means{10, 5, 6};
        std::vector<cv::Matx33d> vars{
            4 * cv::Matx33d::eye(), 3 * cv::Matx33d::eye(), 4 *
       cv::Matx33d::eye()}; std::vector<double> weights{0.3, 0.5, 0.2};
        test_fit_model(means, vars, weights);

        // test1 fundamental test
        std::vector<double> means{10, 0, 0};
        std::vector<cv::Matx33d> vars{cv::Matx33d::eye(),
                                      10000 * cv::Matx33d::eye(),
                                      10000 * cv::Matx33d::eye()};
        std::vector<double> weights{1, 0.0, 0.0};
        test_fit_model(means, vars, weights);

        // test2
        std::vector<double> means{10, 10, 0};
        std::vector<cv::Matx33d> vars{4 * cv::Matx33d::eye(),
                                      3 * cv::Matx33d::eye(),
                                      10000 * cv::Matx33d::eye()};
        std::vector<double> weights{0.5, 0.5, 0.0};
        test_fit_model(means, vars, weights);
        return 0; */
    return 0;
}

/* void test_fit_model(std::vector<double>& means, std::vector<cv::Matx33d>&
vars, std::vector<double>& weights, double a, double alpha, double
fore_threshold) {
    // generate gmm samples

    std::vector<double> samples = generate_gmm_data(means, vars, weights);
    // declare gmm

    gmm::ConfigParam config_param(a, alpha, fore_threshold);
    gmm::GMM gmm(3, config_param);

    for (const auto& sample : samples) {
        gmm.add_sample(sample);
        gmm::ModelParam model_param = gmm.get_model_param();
    }
    std::cout << model_param << std::endl;
} */

std::vector<double> generate_gmm_data(std::vector<double>& means,
                                      std::vector<cv::Matx33d>& vars,
                                      std::vector<double>& weights) {
    assert(means.size() == vars.size() && means.size() == weights.size());

    int sz = means.size();
    for (int i = 0; i < sz; i++) {
        std::random_device rd{};
        std::mt19937 gen{rd()};
        std::normal_distribution<double> d1{100.0, 2.0};
        std::normal_distribution<double> d2{10.0, 2.0};
        std::cout << "random num a: " << d1(gen);
        std::cout << "random num b: " << d2(gen);
    }

    return {};
}