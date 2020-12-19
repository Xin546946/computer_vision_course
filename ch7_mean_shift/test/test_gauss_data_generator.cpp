#include "math_utils.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
void draw_circles(const std::vector<std::array<double, 2>>& data, cv::Mat img, cv::Scalar bgr) {
    for (auto d : data) {
        cv::Point2d p(d[0], d[1]);
        cv::circle(img, p, 1, bgr, 2);
    }
}

int main(int argc, char** argv) {
    auto data1 = generate_gauss_data<double, 2>(1000, {500, 500}, {100, 50});
    auto data2 = generate_gauss_data<double, 2>(1000, {800, 800}, {30, 60});

    auto all_data = data1;
    std::copy(data2.begin(), data2.end(), std::back_inserter(all_data));

    cv::Mat img = cv::Mat::zeros(cv::Size(1000, 1000), CV_8UC3);

    double r_square = 200 * 200;

    while (true) {
        cv::Mat vis = cv::Mat::zeros(cv::Size(1000, 1000), CV_8UC3);
        draw_circles(all_data, vis, cv::Scalar(0, 255, 0));

        cv::imshow("img", vis);
        cv::waitKey(0);
        for (int i = 0; i < all_data.size(); i++) {
            double acc_x = 0.0;
            double acc_y = 0.0;
            int num = 0;
            auto d1 = all_data[i];
            for (int j = 0; j < all_data.size(); j++) {
                auto d2 = all_data[j];

                if (std::pow(d1[0] - d2[0], 2.0) + std::pow(d1[1] - d2[1], 2.0) > r_square) continue;

                acc_x += d2[0];
                acc_y += d2[1];
                num++;
            }

            all_data[i] = {acc_x / num, acc_y / num};
        }
    }

    return 0;
}