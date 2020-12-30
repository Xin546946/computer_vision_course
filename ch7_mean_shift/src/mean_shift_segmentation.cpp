#include "mean_shift_segmentation.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

inline bool is_in_radius(cv::Vec2d center, cv::Vec2d sample, double radius_square) {
    return cv::norm(center - sample, cv::NORM_L2SQR) < radius_square;
}

bool is_convergent(cv::Mat curr, cv::Mat last) {
    if (last.empty()) return false;

    assert(curr.rows == last.rows && curr.cols == last.cols);

    for (int r = 0; r < curr.rows; r++) {
        for (int c = 0; c < curr.cols; c++) {
            if (!is_in_radius(curr.at<cv::Vec2d>(r, c), last.at<cv::Vec2d>(r, c), 0.1)) {
                return false;
            }
        }
    }
    return true;
}

void MeanShiftSeg::process(cv::Mat img) {
    assert(img.type() == CV_8UC3);
    vis_img_origin_ = img.clone();

    cv::Mat img_hsv;
    cv::cvtColor(img, img_hsv, CV_BGR2HSV);

    std::vector<cv::Mat> channels;
    cv::split(img_hsv, channels);

    value_channel_origin_ = channels.back();
    channels.pop_back();

    cv::merge(channels, features_curr_);
    features_curr_.convertTo(features_curr_, CV_64FC2);
    assert(features_curr_.type() == CV_64FC2);
    features_origin_ = features_curr_.clone();

    init_visualization();

    int it = 0;
    cv::Mat features_last;
    while (it++ < 50 && !is_convergent(features_curr_, features_last)) {
        std::cout << "curr iteration : " << it << '\n';
        features_last = features_curr_.clone();
        update_mass_center();
    }
}

void MeanShiftSeg::update_mass_center() {
    visualize();
    for (int r_curr = 0; r_curr < features_curr_.rows; r_curr++) {
        for (int c_curr = 0; c_curr < features_curr_.cols; c_curr++) {
            cv::Vec2d& f_curr = features_curr_.at<cv::Vec2d>(r_curr, c_curr);

            cv::Vec2d sum_f_origin_in_window(0.0, 0.0);
            int num_f_origin_in_window = 0;

            for (int r_ori = 0; r_ori < features_origin_.rows; r_ori++) {
                for (int c_ori = 0; c_ori < features_origin_.cols; c_ori++) {
                    cv::Vec2d f_ori = features_origin_.at<cv::Vec2d>(r_ori, c_ori);

                    if (is_in_radius(f_curr, f_ori, radius_square_)) {
                        sum_f_origin_in_window += f_ori;
                        num_f_origin_in_window++;
                    }
                }
            }
            f_curr = sum_f_origin_in_window / static_cast<double>(num_f_origin_in_window);
        }
    }
}

MeanShiftSeg::MeanShiftSeg(double radius) : radius_square_(radius * radius) {
}

void MeanShiftSeg::init_visualization() {
    vis_feature_space_ = cv::Mat(cv::Size(255, 255), CV_8UC3);
    for (int r = 0; r < features_origin_.rows; r++) {
        for (int c = 0; c < features_origin_.cols; c++) {
            cv::circle(vis_feature_space_, cv::Point(features_origin_.at<cv::Vec2d>(r, c)), 1, cv::Scalar(255, 0, 0),
                       1);
        }
    }
}

void MeanShiftSeg::visualize() {
    cv::Mat vis = vis_feature_space_.clone();

    for (int r = 0; r < vis.rows; r++) {
        for (int c = 0; c < vis.cols; c++) {
            cv::circle(vis, cv::Point(features_curr_.at<cv::Vec2d>(r, c)), 1, cv::Scalar(0, 255, 0), 1);
        }
    }

    cv::imshow("feature space", vis);
    cv::waitKey(1);
}
