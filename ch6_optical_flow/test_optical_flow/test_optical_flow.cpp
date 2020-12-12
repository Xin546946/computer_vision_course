#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/video/video.hpp>
#include <vector>

int main(int argc, char** argv) {
    // open the camera of your laptop and extract feature points
    cv::VideoCapture video(0);
    cv::Mat img_prev;
    std::cout << img_prev.type() << '\n';
    video >> img_prev;
    cv::namedWindow("Feature Points");

    cv::Mat img_gray_prev;
    cv::cvtColor(img_prev, img_gray_prev, CV_RGB2GRAY);

    std::vector<cv::Point2f> feature_points_prev;
    cv::goodFeaturesToTrack(img_gray_prev, feature_points_prev, 50, 0.01, 10, cv::Mat());
    for (cv::Point2f point : feature_points_prev) {
        cv::circle(img_prev, point, 1, cv::Scalar(0, 0, 255), 1, 8, 0);
    }
    std::vector<cv::Point2f>& feature_points_cp = feature_points_prev;
    cv::imshow("Feature Points", img_prev);

    cv::Mat img_curr, img_gray_curr;
    std::vector<cv::Point2f> feature_points_curr;
    std::vector<uchar> status;
    std::vector<float> err;
    while (true) {
        if (cv::waitKey(33) == ' ') {
            break;
        }
        video >> img_curr;
        cv::cvtColor(img_curr, img_gray_curr, CV_RGB2GRAY);

        cv::calcOpticalFlowPyrLK(img_gray_prev, img_gray_curr, feature_points_prev, feature_points_curr, status, err,
                                 cv::Size(21, 21), 3);
        for (int i = 0; i < feature_points_curr.size(); i++) {
            cv::circle(img_curr, feature_points_curr[i], 1, cv::Scalar(0, 0, 255), 2, 8, 0);
            cv::line(img_curr, feature_points_cp[i], feature_points_curr[i], cv::Scalar(255, 0, 0), 2);
        }
        cv::imshow("Feature Points", img_curr);
        swap(feature_points_prev, feature_points_curr);
        img_gray_prev = img_gray_curr.clone();
    }
    return 0;
}