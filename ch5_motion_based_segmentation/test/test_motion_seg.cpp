#include "motion_seg.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

int main(int argc, char** argv) {
    std::vector<cv::Mat> video;
    for (int id = 1; id < 220; id++) {
        cv::Mat img = cv::imread(argv[1] + std::to_string(id) + ".bmp", cv::IMREAD_GRAYSCALE);
        video.push_back(img);
    }

    double sigma_scale = 2.5;
    double update_rate = 0.005;
    double backgroud_ratio = 0.7;

    gmm::ConfigParam config_param(sigma_scale, update_rate, backgroud_ratio);

    int num_gausian = 4;
    MotionSeg ms(video[0].rows, video[0].cols, num_gausian, config_param);
    ms.process(video);

    return 0;
}