
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

cv::Mat compute_gaussian_kernel(int width, int height, double sigma) {
    cv::Mat result = cv::Mat::zeros(cv::Size(width, height), CV_64F);
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            result.at<double>(r, c) =
                (M_1_PI * 0.5 / (sigma * sigma)) * exp(-(pow(r, 2) + pow(c, 2)) / (2 * sigma * sigma));
        }
    }
    return result / cv::sum(result)[0];
}

int get_bin(int gray_value, int width_bin) {
    return gray_value / width_bin;
}

std::vector<int> compute_histogram(int num_bin, cv::Mat img, cv::Mat weight) {
    assert(img.rows == weight.rows && img.cols == weight.cols);
    cv::Mat smooth_img = img.mul(weight);
    std::vector<int> result(num_bin, 0);
    int width_bin = std::ceil(255 / num_bin);
    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            int bin = get_bin(static_cast<float>(img.at<uchar>(r, c)), width_bin);
            result[bin]++;
        }
    }
    return result;
}

int main(int argc, char** argv) {
    cv::Mat img = cv::Mat::ones(cv::Size(100, 100), CV_8UC1);
    cv::Mat weight = compute_gaussian_kernel(100, 100, 10);
    cv::imshow("gaussian", weight);
    cv::waitKey(0);
    std::vector<int> result = compute_histogram(10, img, weight);
    for (auto r : result) {
        std::cout << r << " ";
    }
    std::cout << '\n';
    return 0;
}