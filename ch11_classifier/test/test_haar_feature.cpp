#include "haar_feature.h"
#include "opencv_utils.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
int main(int argc, char** argv) {
    cv::Mat img = read_img(argv[1], cv::IMREAD_GRAYSCALE);
    cv::Mat temp = read_img(argv[2], cv::IMREAD_GRAYSCALE);

    DetectionWindow detection_window(temp.cols, temp.rows);

    detection_window.add_haar_rect(0.200, 0.150, 0.618, 0.317, (cv::Mat_<int>(1, 3) << -1, 1, -1));
    detection_window.add_haar_rect(0.200, 0.330, 0.600, 0.233, (cv::Mat_<int>(2, 1) << 1, -1));
    detection_window.add_haar_rect(0.400, 0.567, 0.300, 0.133, (cv::Mat_<int>(2, 2) << 1, -1, -1, 1));
    detection_window.add_haar_rect(0.360, 0.720, 0.340, 0.15, (cv::Mat_<int>(2, 1) << 1, -1));
    // show all sub img for testing visualization
    detection_window.show_all_sub_image(temp, 0, 0);
    return 0;
}