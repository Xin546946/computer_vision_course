#pragma once
#include "sdf_map.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

void disp_image(cv::Mat& img, cv::String windowName);
void disp_image(cv::Mat& img, cv::String windowName, int delay);
void draw_optical_flow(cv::Mat& fx, cv::Mat& fy, cv::Mat& cflowmap, int step,
                       double scaleFactor, cv::Scalar& color);
// void display_gvf(cv::Mat fx, cv::Mat fy, int delay, bool save);
// void display_contour(cv::Mat img, Contour& contour, int delay);

/**
 * @brief Map image in a coloful space and draw a contour
 *
 * @param sdf_map
 * @return cv::Mat
 */
cv::Mat draw_sdf_map(const SDFMap& sdf_map);  // todo use draw_contour function
cv::Mat apply_jetmap(cv::Mat image);
cv::Mat draw_points(cv::Mat img, cv::Mat points, cv::Scalar color);