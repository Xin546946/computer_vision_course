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

std::vector<float> compute_histogram(int num_bin, cv::Mat img, cv::Mat weight) {
    assert(img.rows == weight.rows && img.cols == weight.cols);
    cv::Mat smooth_img = img.mul(weight);
    std::vector<float> result(num_bin, 0.0f);
    int width_bin = std::ceil(255 / num_bin);
    cv::Mat img_find_bin = smooth_img.clone() * 255.0f;

    img_find_bin.convertTo(smooth_img, CV_8UC1);
    cv::imshow("8UC1 Img", img_find_bin);
    cv::waitKey(0);
    // assert(img_find_bin.type() == CV_8UC1);
    // std::cout << img_find_bin << '\n';

    for (int r = 0; r < img_find_bin.rows; r++) {
        for (int c = 0; c < img_find_bin.cols; c++) {
            int gray_value = static_cast<int>(img_find_bin.at<uchar>(r, c));
            int bin = get_bin(gray_value, width_bin);
            // std::cout << "Gray value is: " << gray_value << ", at Bin " << bin << '\n';
            result[bin] += 1.0;
        }
    }
    float sum = static_cast<float>(std::accumulate(result.begin(), result.end(), 0));
    std::vector<float> res;
    std::cout << sum << '\n';
    for (float& bin_val : result) {
        bin_val /= sum;
        res.push_back(bin_val);
    }
    return res;
}

int main(int argc, char** argv) {
    cv::Mat img = cv::imread("/home/kit/computer_vision_course/ch7_mean_shift/img/segmentation/horse.jpg", 0);
    img.convertTo(img, CV_64F);
    cv::Mat gaussian = get_gaussian_kernel(img.cols, img.rows, 100);
    std::vector<float> hist = compute_histogram(10, img, gaussian);
    for (int val : hist) {
        std::cout << val << " ";
    }
    float sum = static_cast<float>(std::accumulate(hist.begin(), hist.end(), 0));
    std::cout << "Sum of hist " << sum << " should equal to " << 1 << '\n';
    std::cout << '\n';
    return 0;
}