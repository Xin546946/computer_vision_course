#include "hough.h"
#include "opencv_utils.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    // cv::Mat img = read_img(argv[1], cv::IMREAD_GRAYSCALE);

    cv::Mat img(cv::Mat::ones(100, 100, CV_8UC1) * 255);

    cv::line(img, cv::Point(0, 30), cv::Point(99, 30), cv::Scalar(0, 0, 0), 5);
    img.convertTo(img, CV_8UC1);
    // cv::imshow("xx", img);
    // cv::waitKey(0);

    std::vector<LineParam> line_param = line_detection(img);
    draw_line(img, line_param);
    return 0;
}