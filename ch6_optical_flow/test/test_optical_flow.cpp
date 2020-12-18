#include "opencv_utils.h"
#include "optical_flow.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

cv::Mat generate_img_with_rect(int rows, int cols, cv::Point2f ul, int width, int height);

// void vis_optical_flow();
std::vector<cv::Point2f> optical_flow_track(cv::Mat img1, cv::Mat img2, std::vector<cv::Point2f> prev_fps) {
    // cv::goodFeaturesToTrack(img1, prev_fps, 100, 0.01, 3);
    cv::Mat img2_clone = img2.clone();
    cv::cvtColor(img2_clone, img2_clone, CV_GRAY2BGR);

    OpticalFlow optical_flow(img1, img2, prev_fps, cv::Size2i(21, 21));
    std::vector<cv::Point2f> curr_fps = optical_flow.get_curr_fps();

    for (cv::Point2f fp : curr_fps) {
        cv::circle(img2_clone, fp, 1, cv::Scalar(0, 255, 0), 2, 8, 0);
    }
    cv::imshow("Feature Points", img2_clone);
    cv::waitKey(1);
    return curr_fps;
}

int main(int argc, char** argv) {
    std::vector<cv::Mat> video;
    for (int id = 1; id < 200; id++) {
        cv::Mat img = cv::imread(argv[1] + std::to_string(id) + ".jpg", cv::IMREAD_GRAYSCALE);
        assert(!img.empty());
        video.push_back(img);
    }
    std::vector<cv::Point2f> curr_fps;
    cv::goodFeaturesToTrack(video[0], curr_fps, 200, 0.2, 3);
    cv::Mat prev_img = video[0];
    for (int i = 1; i < video.size(); i++) {
        std::vector<cv::Point2f> fps = optical_flow_track(prev_img, video[i], curr_fps);
        prev_img = video[i];
        curr_fps = fps;
        std::cout << "The size of feature points: " << fps.size() << '\n';
    }

    return 0;
}

cv::Mat generate_img_with_rect(int rows, int cols, cv::Point2f ul, int width, int height) {
    cv::Mat img = 255 * cv::Mat::ones(cv::Size(cols, rows), CV_32FC1);
    put_val_from_ul(0.0f, img, ul.x, ul.y, width, height);
    return img;
}