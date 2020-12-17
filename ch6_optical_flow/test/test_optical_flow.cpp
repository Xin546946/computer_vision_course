#include "opencv_utils.h"
#include "optical_flow.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

cv::Mat generate_img_with_rect(int rows, int cols, cv::Point2f ul, int width, int height);

int main(int argc, char** argv) {
    cv::Mat img1 = generate_img_with_rect(100, 100, cv::Point2f(10.0f, 10.0f), 10, 10);
    cv::Mat img2 = generate_img_with_rect(100, 100, cv::Point2f(30.0f, 30.0f), 10, 10);
    cv::imshow("img1", img1);
    cv::waitKey(0);
    cv::imshow("img2", img2);
    cv::waitKey(0);

    std::vector<cv::Point2f> prev_fps;
    cv::goodFeaturesToTrack(img1, prev_fps, 10, 0.01, 3);

    std::vector<cv::Point2f> curr_fps;
    std::vector<uchar> status;
    OpticalFlow optical_flow(img1, img2, prev_fps, curr_fps, status, cv::Size(11, 11));
    std::vector<cv::Point2f> fps_optical_flow = optical_flow.get_result();
    optical_flow.vis_optical_flow();

    return 0;
}

cv::Mat generate_img_with_rect(int rows, int cols, cv::Point2f ul, int width, int height) {
    cv::Mat img = 255 * cv::Mat::ones(cv::Size(cols, rows), CV_8UC1);
    put_val_from_ul(0, img, ul.x, ul.y, width, height);
    return img;
}