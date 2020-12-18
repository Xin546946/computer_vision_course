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

    std::vector<uchar> get_status() const;
    std::vector<cv::Point2f> get_curr_fps() const;

   private:
    void process_optical_flow();
    uchar update_status(cv::Matx22f matrix);

    std::vector<cv::Point2f> fps_;
    std::vector<cv::Point2f> curr_fps_;
    cv::Matx22f compute_A(cv::Mat grad_x, cv::Mat grad_y);
    cv::Matx21f compute_b(cv::Mat prev_img, cv::Mat curr_img, cv::Mat grad_x, cv::Mat grad_y);
    cv::Mat img1_;
    cv::Mat img2_;
    cv::Size2i win_size_;
    std::vector<uchar> status_;  // todo check if it is useful
};