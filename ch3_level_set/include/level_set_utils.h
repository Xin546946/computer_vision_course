#pragma once
#include "level_set_cv.h"
#include "sdf_map.h"
#include <opencv2/core.hpp>

// This file is used to be called by CV Model and LBF Model for some common
// functions

/**
 * @brief  if phi>0, H(phi) = 1; if phi<0, H(phi) = 0;
 *
 * @param sdf_map
 * @return cv::Mat
 */

cv::Mat heaviside(const SDFMap& sdf_map, double eps = 1.0);
cv::Mat dirac(const SDFMap& sdf_map, double eps = 1.0);
cv::Mat complementary_heaviside(const SDFMap& sdf_map,
                                double eps = 1.0);  // 1-Heaciside
// todo define a heaviside function according to the std::for_each
// todo using overload function for heaviside function

double compute_length_energy(const SDFMap& sdf_map);
cv::Mat computer_div_delta_map(const SDFMap& sdf_map);

cv::Mat compute_derivative_data_term(const SDFMap& sdf_map,
                                     cv::Mat original_image,
                                     double weight_foreground,
                                     double weight_background,
                                     double center_foreground,
                                     double center_background, double eps);
cv::Mat compute_derivative_length_term(const SDFMap& sdf_map, double eps);
cv::Mat compute_derivative_gradient_term(const SDFMap& sdf_map);
cv::Mat compute_laplacian_map(const SDFMap& sdf_map);
cv::Mat gaussian_kernel(int size, double sigma);
cv::Mat compute_foreground_center();
double compute_center(cv::Mat img, const SDFMap& sdf_map, double eps,
                      bool is_background);
cv::Mat compute_mat_grad_magnitude(cv::Mat mat);
double compute_data_term_energy(const SDFMap& sdf_map, cv::Mat original_image,
                                double weight_foreground,
                                double weight_background,
                                double center_foreground,
                                double center_background, double eps);
double compute_length_term_energy(const SDFMap& sdf_map, double eps);
double compute_gradient_preserve_energy(const SDFMap& sdf_map);