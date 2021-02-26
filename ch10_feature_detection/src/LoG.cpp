/**
______________________________________________________________________
*********************************************************************
* @brief This file is developed for the course of ShenLan XueYuan:
* Fundamental implementations of Computer Vision
* all rights preserved
* @author Xin Jin, Zhaoran Wu
* @contact: xinjin1109@gmail.com, zhaoran.wu1@gmail.com
*
______________________________________________________________________
*********************************************************************
**/
#include "LoG.h"
#include "opencv_utils.h"

void LoG::run() {
    // step 1: gaussian smoothing
    cv::GaussianBlur(img_, result_, param_gaussian_.size_, param_gaussian_.sigma_x_, param_gaussian_.sigma_y_,
                     cv::BORDER_DEFAULT);
    cv::Mat result_gray;
    cv::cvtColor(result_, result_gray, cv::COLOR_BGR2GRAY);
    cv::imshow("gray value", result_gray);
    cv::waitKey(0);

    // step 2: laplacian operator
    cv::Mat lap_dist;
    cv::Laplacian(result_gray, lap_dist, param_laplacian_.ddepth_, param_laplacian_.kernel_size_,
                  param_laplacian_.scale_, param_laplacian_.delta_, cv::BORDER_DEFAULT);
    // cv::Laplacian(result_gray, lap_dist, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT);
    // cv::convertScaleAbs(lap_dist, lap_dist);
    lap_dist.convertTo(lap_dist, CV_8UC1);

    // cv::Mat vis_lap = get_float_mat_vis_img(lap_dist);
    cv::imshow("Laplacian", lap_dist);
    cv::waitKey(0);
    // step 3: threshold for binary image
    cv::Mat thres_dist;
    cv::threshold(lap_dist, thres_dist, 0.0, param_thres_.max_val_, param_thres_.threshold_type_);

    cv::imshow("threshold to binary img", thres_dist);
    cv::waitKey(0);
    // step 4: erosion and dilation
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
    cv::Mat erosion_dist, dilation_dist;
    cv::dilate(thres_dist, dilation_dist, element, cv::Point(-1, -1), 1);   // 膨胀
    cv::erode(dilation_dist, erosion_dist, element, cv::Point(-1, -1), 1);  // 腐蚀
    cv::imshow("Result", erosion_dist);
    cv::waitKey(0);
}