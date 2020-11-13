
#include "sdf_map.h"
#include <opencv2/core.hpp>

/**
 * @brief  if phi>0, H(phi) = 1; if phi<0, H(phi) = 0;
 *
 * @param sdf_map
 * @return cv::Mat
 */
cv::Mat heaviside(const SDFMap& sdf_map, double eps = 1.0);

cv::Mat complementary_heaviside(const SDFMap& sdf_map,
                                double eps = 1.0);  // 1-Heaciside
// todo define a heaviside function according to the std::for_each
// todo using overload function for heaviside function

double heaciside_derivative(double z, double eps = 1.0);
double get_length(SDFMap sdf_map);
cv::Mat compute_div(cv::Mat vec);
