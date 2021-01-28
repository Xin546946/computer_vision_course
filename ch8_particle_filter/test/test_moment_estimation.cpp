#include "math_utils.h"
#include <complex>
#include <iostream>
#include <random>

int main(int argc, char** argv) {
    double p1 = 1.0;
    double p2 = 2.0;
    double p3 = 2.15;

    double sum1 = 0.0;
    double sum2 = 0.0;

    std::complex<double> sum3(0.0, 0.0);

    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<double> normal_dists(0.0, 1.0);

    for (int i = 1; i < 1e6; i++) {
        double norm_data = normal_dists(gen);
        sum1 += std::pow(norm_data, p1);
        sum2 += std::pow(norm_data, p2);
        std::complex<double> tmp(norm_data, 0.0);
        sum3 += std::pow(tmp, p3);
        // std::cout << sum3 << '\n';
    }
    sum1 /= 1e6;
    sum2 /= 1e6;
    sum3 /= 1e6;
    std::cout << "The " << p1 << "-th moments of normal distribution is: " << sum1 << '\n';
    std::cout << "The " << p2 << "-th moments of normal distribution is: " << sum2 << '\n';
    std::cout << "The " << p3 << "-th moments of normal distribution is: " << sum3 << '\n';
    return 0;
}