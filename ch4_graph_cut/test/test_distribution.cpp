#include "display.h"
#include "distribution.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    // cv::Mat image = cv::imread(argv[1], cv::IMREAD_COLOR);
    cv::Mat img = cv::imread(
        "/home/kit/computer_vision_course/ch4_graph_cut/img/index.jpeg");
    cv::resize(img, img, cv::Size(20, 20));
    cv::cvtColor(img, img, CV_8UC3);
    std::vector<cv::Point> foreground{cv::Point(0, 0)};
    std::vector<cv::Point> background{cv::Point(3, 3)};
    Distribution distribution(img, foreground, background);
    // test all fore- and background weight
    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            double fore_weight = distribution.get_weight(
                r, c, 0);  // foreground_,background_ private variable;
            double back_weight = distribution.get_weight(r, c, 1);
            std::cout << "foreground weight at (" << r << ", " << c << ")"
                      << " is " << fore_weight << '\n';
            std::cout << "background weight at (" << r << ", " << c << ")"
                      << " is " << back_weight << '\n';
        }
    }
    // get probability map
    cv::Mat fore_img = distribution.get_probability_map(0);
    cv::Mat back_img = distribution.get_probability_map(1);
    // visualize the probability map
    get_float_mat_vis_img(fore_img);
    get_float_mat_vis_img(back_img);
    return 0;
}