#include "math_utils.h"

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    auto data1 = generate_gauss_data<double, 2>(1000, {500, 500}, {100, 50});
    auto data2 = generate_gauss_data<double, 2>(1000, {80, 800}, {30, 30});

    cv::Mat img = cv::Mat::zeros(cv::Size(1000, 1000), CV_8UC3);

    for (auto d : data1) {
        cv::Point2d p(d[0], d[1]);
        cv::circle(img, p, 1, cv::Scalar(0, 255, 0), 2);
    }

    for (auto d : data2) {
        cv::Point2d p(d[0], d[1]);
        cv::circle(img, p, 1, cv::Scalar(255, 0, 0), 2);
    }

    cv::imshow("img", img);
    cv::waitKey(0);

    return 0;
}