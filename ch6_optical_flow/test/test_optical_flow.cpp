#include "opencv_utils.h"
#include "optical_flow.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

cv::Mat generate_img_with_rect(int rows, int cols, cv::Point2f ul, int width, int height);

// void vis_optical_flow();

int main(int argc, char** argv) {
    // cv::Mat img1 = generate_img_with_rect(1000, 1000, cv::Point2f(100.0f, 100.0f), 100, 100);
    // cv::Mat img2 = generate_img_with_rect(1000, 1000, cv::Point2f(120.0f, 120.0f), 100, 100);
    // cv::GaussianBlur(img1, img1, cv::Size(31, 31), 31, 31);
    // cv::GaussianBlur(img2, img2, cv::Size(31, 31), 31, 31);
    cv::Mat img1 = cv::imread(argv[1], CV_8UC1);
    cv::Mat img2 = cv::imread(argv[2], CV_8UC1);
    cv::imshow("img1", img1);
    cv::waitKey(0);
    cv::imshow("img2", img2);
    cv::waitKey(0);

    std::vector<cv::Point2f> prev_fps;
    cv::goodFeaturesToTrack(img1, prev_fps, 100, 0.01, 3);
    cv::Mat img2_clone = img2.clone();
    cv::cvtColor(img2_clone, img2_clone, CV_GRAY2BGR);

    for (cv::Point2f fp : prev_fps) {
        cv::circle(img2_clone, fp, 1, cv::Scalar(0, 0, 255), 2, 8, 0);
    }
    cv::imshow("Feature Points", img2_clone);
    cv::waitKey(0);
    OpticalFlow optical_flow(img1, img2, prev_fps, cv::Size2i(55, 55));
    std::vector<cv::Point2f> curr_fps = optical_flow.get_curr_fps();
    std::vector<uchar> status = optical_flow.get_status();
    std::cout << "******Print status**********" << '\n';
    for (uchar c : status) {
        std::cout << static_cast<int>(c) << " ";
    }
    std::cout << '\n';
    std::cout << "feature_points_size: " << curr_fps.size() << '\n';
    for (cv::Point2f fp : curr_fps) {
        std::cout << "feature point is at " << fp.x << " " << fp.y << '\n';
        cv::circle(img2_clone, fp, 1, cv::Scalar(0, 255, 0), 2, 8, 0);
    }
    cv::imshow("Feature Points", img2_clone);
    cv::waitKey(0);

    return 0;
}

cv::Mat generate_img_with_rect(int rows, int cols, cv::Point2f ul, int width, int height) {
    cv::Mat img = 255 * cv::Mat::ones(cv::Size(cols, rows), CV_32FC1);
    put_val_from_ul(0.0f, img, ul.x, ul.y, width, height);
    return img;
}