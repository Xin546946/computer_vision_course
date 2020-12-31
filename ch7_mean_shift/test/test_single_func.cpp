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
int get_bin(float gray_value, int width_bin) {
    return gray_value / width_bin;
}
cv::Mat compute_gaussian_kernel(int width, int height, double sigma) {
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

cv::Mat compute_back_projection(cv::Mat img, std::vector<double> hist_temp, std::vector<double> hist_candidate) {
    assert(hist_candidate.size() == hist_temp.size());
    cv::Mat result = cv::Mat::zeros(img.size(), CV_64F);
    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            int bin = get_bin(static_cast<float>(img.at<double>(r, c)), std::ceil(255 / hist_temp.size()));

            result.at<double>(r, c) = std::sqrt(hist_temp[bin] / (hist_candidate[bin] + 1e-8));
        }
    }
    return result;
}

std::vector<double> compute_histogram(int num_bin, cv::Mat img, cv::Mat weight) {
    assert(img.rows == weight.rows && img.cols == weight.cols);
    std::vector<double> result(num_bin, 0.0);
    int width_bin = std::ceil(255 / num_bin);

    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            int gray_value = static_cast<int>(img.at<uchar>(r, c));
            int bin = get_bin(gray_value, width_bin);
            std::cout << "Gray value is: " << gray_value << ", at Bin " << bin << '\n';
            result[bin] += weight.at<double>(r, c);
        }
    }
    double sum = std::accumulate(result.begin(), result.end(), 0.0);

    std::cout << sum << '\n';
    for (double& bin_val : result) {
        bin_val /= sum;
    }
    return result;
}

int main(int argc, char** argv) {
    cv::Mat img = cv::imread("/home/kit/computer_vision_course/ch7_mean_shift/img/segmentation/horse.jpg", 0);

    img.convertTo(img, CV_64F);
    cv::Mat gaussian = compute_gaussian_kernel(img.cols, img.rows, 100);
    std::vector<double> hist_temp = compute_histogram(10, img, gaussian);
    std::vector<double> hist_candidate = compute_histogram(10, img, gaussian);
    cv::Mat result = compute_back_projection(img, hist_temp, hist_temp);
    cv::Mat vis = get_float_mat_vis_img(result);
    cv::imshow("back_projection", vis);
    cv::waitKey(0);

    return 0;
}