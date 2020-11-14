#include "level_set_helper_function.h"

inline double heaviside(double z, double eps) {
    return 0.5 * (1 + (2 / M_PI) * atan2(z, eps));
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