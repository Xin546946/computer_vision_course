#include "level_set_helper_function.h"
// #include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

cv::Mat computer_div_delta_map(const SDFMap& sdf_map) {
    cv::Mat phi = sdf_map.map_;
    cv::Mat phi_dev_x, phi_dev_y;
    cv::Sobel(phi, phi_dev_x, CV_64F, 1, 0, 3);
    cv::Sobel(phi, phi_dev_y, CV_64F, 0, 1, 3);
    cv::Mat phi_dev_xx, phi_dev_xy, phi_dev_yy;
    cv::Sobel(phi_dev_x, phi_dev_xx, CV_64F, 1, 0, 3);
    cv::Sobel(phi_dev_x, phi_dev_xy, CV_64F, 0, 1, 3);
    cv::Sobel(phi_dev_y, phi_dev_yy, CV_64F, 0, 1, 3);
    cv::Mat pow_phi_dev_x, pow_phi_dev_y, pow_phi_denominator;
    cv::pow(phi_dev_x, 2, pow_phi_dev_x);
    cv::pow(phi_dev_y, 2, pow_phi_dev_y);
    cv::pow(phi_dev_x.mul(phi_dev_x) + phi_dev_y.mul(phi_dev_y), -2 / 3,
            pow_phi_denominator);
    return (phi_dev_xx.mul(phi_dev_y.mul(phi_dev_y)) -
            2 * phi_dev_x.mul(phi_dev_y.mul(phi_dev_xy)) +
            phi_dev_yy.mul(phi_dev_x.mul(phi_dev_x)))
        .mul(pow_phi_denominator);
}

inline double heaviside(double z, double eps) {
    return 0.5 * (1 + M_2_PI * atan2(z, eps));
}

inline double dirac(double z, double eps = 1.0) {
    return eps * M_1_PI / (std::pow(eps, 2.0) + std::pow(z, 2.0));
}

cv::Mat heaviside(const SDFMap& sdf_map, double eps) {
    cv::Mat phi = sdf_map.map_;
    cv::Mat result(phi.size(), phi.type());
    std::transform(phi.begin<double>(), phi.end<double>(),
                   result.begin<double>(),
                   [=](double z) { return heaviside(z, eps); });

    return result;
}

cv::Mat complementary_heaviside(const SDFMap& sdf_map, double eps) {
    cv::Mat h_phi = heaviside(sdf_map, eps);
    return cv::Mat::ones(h_phi.size(), h_phi.type()) - h_phi;
}

cv::Mat dirac(const SDFMap& sdf_map, double eps) {
    cv::Mat phi = sdf_map.map_;
    cv::Mat result(phi.size(), phi.type());
    std::transform(phi.begin<double>(), phi.end<double>(),
                   result.begin<double>(),
                   [=](double z) { return dirac(z, eps); });
    return result;
}

double compute_length_energy(SDFMap sdf_map, double eps) {
    cv::Mat heaviside_map = heaviside(sdf_map, eps);
    return cv::sum(heaviside_map)[0];
}