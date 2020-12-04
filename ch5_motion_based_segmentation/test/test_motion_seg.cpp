#include "motion_seg.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    std::vector<cv::Mat> video;
    for (int id = 1; id < 220; id++) {
        cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
        video.push_back(img);
    }

    double a = 1.5;
    double alpha = 0.01;
    double fore_threshold = 0.25;

    gmm::ConfigParam config_param(a, alpha, fore_threshold);

    int num_gausian = 3;
    MotionSeg ms(video[0].rows, video[0].cols, num_gausian, config_param);
    ms.process(video);

    return 0;
}