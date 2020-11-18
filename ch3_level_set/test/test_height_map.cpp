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

#include "display.h"
#include "height_map.h"
#include "level_set_utils.h"
#include <opencv2/core.hpp>

int main(int argc, char** argv) {
    // define and and initialize a sdf_map object
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);

    assert(img.channels() == 1);

    // disp_image(div_img, "divergence", 0);
    int rows = img.rows;
    int cols = img.cols;
    cv::Point2d center(cols / 2.f, rows / 2.f);
    double radius = std::min(rows, cols) / 4.f;
    // HightMap sdf_map(rows, cols, center, radius);
    HightMap sdf_map(rows, cols);
    cv::Mat div = compute_div_delta_map(sdf_map);
    disp_image(div, "divergence", 0);
    cv::Mat sdf_draw = draw_sdf_map(sdf_map);
    cv::Mat sdf_with_contour = draw_points(
        sdf_draw, sdf_map.get_contour_points(), cv::Scalar(255, 255, 255));
    disp_image(sdf_with_contour, "sdf", 0);

    cv::Mat fore_back_ground = sdf_map.get_fore_background_label_map();
    disp_image(fore_back_ground, "fore- and background", 0);

    cv::Mat h_phi = heaviside(sdf_map, 50);
    disp_image(h_phi, "h_phi", 0);

    cv::Mat one_minues_h_phi = complementary_heaviside(sdf_map, 50);
    disp_image(one_minues_h_phi, "one_minus_h_phi", 0);

    cv::Mat dirac_phi = dirac(sdf_map, 50.0);
    disp_image(dirac_phi, "dirac", 0);

    double energy_grad_mag_map = sdf_map.get_gradient_magnitude_level_set();
    std::cout << energy_grad_mag_map << std::endl;

    cv::Mat test_derivative_length_term =
        compute_derivative_length_term(sdf_map, 50.0);
    disp_image(test_derivative_length_term, "derivative length term", 0);

    cv::Mat test_laplacian_map = compute_laplacian_map(sdf_map);
    disp_image(test_laplacian_map, "test_laplacian_map", 0);

    cv::Mat test_derivative_gradient_term =
        compute_derivative_gradient_term(sdf_map);
    disp_image(test_derivative_gradient_term, "test_derivative_gradient_term",
               0);

    return 0;
}