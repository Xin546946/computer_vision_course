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
#pragma once
#include "height_map.h"
#include "level_set_cv.h"
#include <opencv2/core.hpp>

// This file is used to be called by CV Model and LBF Model for some common
// functions

/**
 * @brief  if phi>0, H(phi) = 1; if phi<0, H(phi) = 0;
 *
 * @param sdf_map
 * @return cv::Mat
 */

cv::Mat do_sobel(cv::Mat im, int flag);
cv::Mat heaviside(const HightMap& sdf_map, double eps = 1.0);
cv::Mat dirac(const HightMap& sdf_map, double eps = 1.0);
cv::Mat complementary_heaviside(const HightMap& sdf_map,
                                double eps = 1.0);  // 1-Heaciside
cv::Mat compute_div_delta_map(const HightMap& sdf_map);
// todo define a heaviside function according to the std::for_each
// todo using overload function for heaviside function

double compute_length_energy(const HightMap& sdf_map);

cv::Mat compute_derivative_data_term(const HightMap& sdf_map,
                                     cv::Mat original_image,
                                     double weight_foreground,
                                     double weight_background,
                                     double center_foreground,
                                     double center_background, double eps);
cv::Mat compute_derivative_length_term(const HightMap& sdf_map, double eps);
cv::Mat compute_derivative_gradient_term(const HightMap& sdf_map);
cv::Mat compute_laplacian_map(const HightMap& sdf_map);
cv::Mat gaussian_kernel(int size, double sigma);
cv::Mat compute_foreground_center();
double compute_center(cv::Mat img, const HightMap& sdf_map, double eps,
                      bool is_background);
double compute_center_in_window(int row, int col, int size,
                                cv::Mat gauss_kernel, cv::Mat img,
                                const HightMap& sdf_map, double eps,
                                bool is_background);
cv::Mat compute_mat_grad_magnitude(cv::Mat mat);
double compute_data_term_energy(const HightMap& sdf_map, cv::Mat original_image,
                                double weight_foreground,
                                double weight_background,
                                double center_foreground,
                                double center_background, double eps);
double compute_length_term_energy(const HightMap& sdf_map, double eps);
double compute_gradient_preserve_energy(const HightMap& sdf_map);
cv::Mat compute_square_diff(cv::Mat img1, cv::Mat img2);
cv::Mat get_sub_image(cv::Mat image, int row, int col, int window_size);
void visualize_lvl_set_segemenation(cv::Mat origin_img, const HightMap& phi,
                                    int delay = 0);