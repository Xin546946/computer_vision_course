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
#pragma once
#include "opencv_utils.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * @brief class Canny for edge detection
 *
 */
class Canny {
   public:
    Canny(cv::Mat img) : img_(img.clone()), img_64f_(img.clone()), result_(cv::Mat::zeros(img.size(), CV_64FC1)) {
        img_.convertTo(img_64f_, CV_64FC1);
    }
    void run();
    cv::Mat show_result() {
        cv::Mat vis = get_float_mat_vis_img(result_);
        imshow("Canny edge detector", vis);
        cv::waitKey(0);
    }

   private:
    cv::Mat img_;
    cv::Mat img_64f_;
    cv::Mat result_;
};