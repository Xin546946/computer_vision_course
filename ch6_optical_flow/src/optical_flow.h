#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

class OpticalFlow {
   public:
    /**
     * @brief Given two consecutive imgs to calculate optical flow
     *
     * @param img1
     * @param img2
     */
    OpticalFlow(cv::Mat img1, cv::Mat img2, const std::vector<cv::Point2f>& prev_fps,
                cv::Size2i win_size = cv::Size(11, 11));
    std::vector<cv::Point2f> compute_curr_fps();
    std::vector<uchar> get_status() const;

   private:
    cv::Point2f compute_flow_in_window(cv::Point2f feature_point);
    uchar update_status();
    std::vector<cv::Point2f> fps_;
    cv::Mat img1_;
    cv::Mat img2_;
    cv::Size2i win_size_;
};