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
    OpticalFlow(cv::Mat img1, cv::Mat img2, std::vector<cv::Point2f> prev_fps, std::vector<cv::Point2f> curr_fps,
                std::vector<uchar> status, cv::Size2i win_size);
    /**
     * @brief Get the result fps
     *
     * @return std::vector<cv::Point2f>
     */
    std::vector<cv::Point2f> get_result() const {
        return fps_;
    };
    void vis_optical_flow();

   private:
    void update_status();
    cv::Point2f compute_flow_in_window(cv::Point2f feature_point, cv::Size win_size);
    std::vector<cv::Point2f> fps_;
    cv::Mat img1_;
    cv::Mat img2_;
};