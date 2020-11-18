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
#include <opencv2/core.hpp>

class SDFMap {  // phi
   public:
    /**
     * @brief Construct a new SDFMap object, Output a SDF
     *
     * @param center
     * @param radius
     */
    SDFMap(int rows, int cols, cv::Point center, double radius);
    SDFMap(int rows, int cols);
    friend cv::Mat heaviside(
        const SDFMap& sdf_map,
        double eps);  // if phi>0, H(phi) = 1; if phi<0, H(phi) = 0;
    friend cv::Mat draw_sdf_map(const SDFMap& sdf_map);
    friend cv::Mat dirac(const SDFMap& sdf_map, double eps);
    friend double compute_length_energy(const SDFMap& sdf_map);
    friend cv::Mat compute_laplacian_map(const SDFMap& sdf_map);
    friend double compute_gradient_preserve_energy(const SDFMap& sdf_map);
    friend cv::Mat compute_div_delta_map(const SDFMap& sdf_map);
    friend double compute_center_in_window(int row, int col, int size,
                                           cv::Mat gauss_kernel, cv::Mat img,
                                           const SDFMap& sdf_map, double eps,
                                           bool is_background);
    // friend cv::Mat compute_mat_grad_magnitude(cv::Mat mat);
    void add(cv::Mat step);

    // void get_length();  // get the total length of all contours

    cv::Mat get_contour_points() const;
    double get_gradient_magnitude_level_set() const;
    cv::Mat get_fore_background_label_map()
        const;  // get segment result, forground background

   private:
    cv::Mat map_;  // cv_64F
};