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
#include "hough.h"
#include "opencv_utils.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    cv::Mat img = read_img(argv[1], cv::IMREAD_GRAYSCALE);
    cv::GaussianBlur(img, img, cv::Size(5, 5), 5, 5);
    // cv::Mat img(cv::Mat::ones(100, 100, CV_8UC1) * 255);
    //
    // cv::line(img, cv::Point(30, 0), cv::Point(60, 100), cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    //// cv::line(img, cv::Point(10, 0), cv::Point(11, 100), cv::Scalar(0, 0, 0), 2);
    //
    // img.convertTo(img, CV_8UC1);
    // cv::imshow("xx", img);
    // cv::waitKey(0);

    std::vector<LineParam> line_param = line_detection(img, 1, 1);
    draw_line(img, line_param);
    return 0;
}