#include "histogram.h"
#include "opencv_utils.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    cv::Mat img = read_img(argv[1], cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = read_img(argv[1], cv::IMREAD_GRAYSCALE);
    img.convertTo(img, CV_64FC1);
    std::cout << "@@@Start to build histogram! " << '\n';
    Histogram histogram(10, 0.0, 255.0);
    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            histogram.add_data(img.at<double>(r, c));
        }
    }
    std::cout << "@@@Finish building histogram! " << '\n';

    int bin = histogram.get_bin(255.0);
    std::cout << "Value 255.0 is at " << bin << "-th histogram!" << '\n';
    double bin_height = histogram.get_bin_height(0);
    std::cout << " The 0-th histogram has height " << bin_height << '\n';
    for (auto his : histogram.get_hist()) {
        std::cout << "The" << bin << "-th histogram has height: " << his << '\n';
    }

    std::cout << "************************" << '\n';
    std::cout << "@@@ Now compute the histogram from opencv function" << '\n';
    int hist_size = 10;
    // hue varies from 0 to 179, see cvtColor

    // saturation varies from 0 (black-gray-white) to
    // 255 (pure spectrum color)
    float range[] = {0, 256};  // the upper boundary is exclusive
    const float* hist_range = {range};

    cv::Mat hist;
    // we compute the histogram from the 0-th and 1-st channels
    int channels[] = {0};
    cv::calcHist(&img2, 1, 0, cv::Mat(),  // do not use mask
                 hist, 1, &hist_size, &hist_range);
    std::cout << hist << '\n';
    return 0;
}