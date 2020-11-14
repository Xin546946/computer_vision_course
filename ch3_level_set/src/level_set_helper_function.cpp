#include "level_set_helper_function.h"

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
