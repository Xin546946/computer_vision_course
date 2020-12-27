#include "data_base.h"
#include "display.h"
#include "memory"
#include "opencv_utils.h"
#include "tracking_data_base.h"
#include "visualizer.h"
#include <algorithm>
#include <chrono>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>

cv::Mat get_gaussian_kernel(int width, int height, double sigma) {
    cv::Point center((width - 1) / 2, (height - 1) / 2);
    cv::Mat result = cv::Mat::zeros(cv::Size(width, height), CV_64F);
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            result.at<double>(r, c) = (M_1_PI * 0.5 / (sigma * sigma)) *
                                      exp(-(pow(r - center.y, 2) + pow(c - center.x, 2)) / (2 * sigma * sigma));
        }
    }
    return result / cv::sum(result)[0];
}

int main(int argc, char** argv) {
    int width = 28;
    int height = 59;
    float sigma = 100;

    cv::Mat gaussian = get_gaussian_kernel(width, height, sigma);
    cv::Mat vis = get_float_mat_vis_img(gaussian);
    // std::cout << vis << '\n';
    cv::imshow("Gaussian", vis);
    cv::waitKey(0);
    cv::Mat gaussian2 = get_gaussian_kernel(59, 59, sigma);
    cv::Mat vis2 = get_float_mat_vis_img(gaussian2);
    // std::cout << vis << '\n';
    cv::imshow("Gaussian2", vis2);
    cv::waitKey(0);

    return 0;
}