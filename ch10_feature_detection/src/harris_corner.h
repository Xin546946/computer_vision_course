#pragma once
#include "opencv_utils.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class HarrisCornerDetector {
   public:
    HarrisCornerDetector(cv::Mat img) : img_(img), img_64f_(img.size(), CV_64FC1), result_(img.clone()) {
        img.convertTo(img_64f_, CV_64FC1);
        cv::GaussianBlur(img_64f_, img_64f_, cv::Size(3, 3), 3.0, 3.0);
        cv::copyMakeBorder(img_64f_, img_64f_, 3, 3, 3, 3, cv::BORDER_CONSTANT, 200);
        // assert(img_64f_.type() == CV_64FC1);
        std::cout << img_64f_.size() << " " << img_.size() << '\n';
        cv::cvtColor(img, result_, cv::COLOR_GRAY2BGR);
        cv::imshow("img color", result_);
        cv::waitKey(0);
    }
    void run();
    void show_result() {
        // cv::Mat vis = get_float_mat_vis_img(result_);
        cv::imshow("Harris Corner Detecter", result_);
        cv::waitKey(0);
    }

   private:
    cv::Mat img_;
    cv::Mat img_64f_;
    cv::Mat result_;
};