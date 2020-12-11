#include "opencv_utils.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    std::vector<cv::Mat> video = record_webcam();
    int i = 0;
    for (cv::Mat img : video) {
        cv::imshow("img", img);
        cv::imwrite(argv[1] + std::to_string(i++) + ".png", img);
        cv::waitKey(1);
    }

    return 0;
}