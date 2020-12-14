#include "opencv_utils.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    cv::Mat img = cv::Mat::zeros(cv::Size(100, 100), CV_8UC1);
    put_val_from_ul(255, img, 10, 10, 40, 40);
    put_val_around(0, img, 20, 20, 11, 11);
    cv::imshow("img", img);
    cv::waitKey(0);
    return 0;
}