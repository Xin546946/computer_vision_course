/**
______________________________________________________________________
*********************************************************************
* @brief This file is developed for the course of ShenLan XueYuan:
* Fundamental implementations of Computer Vision
* all rights preserved
* @author Xin Jin, Zhaoran Wu
* @contact: xinjin1109@gmail.com, zhaoran.wu1@gmail.com
*
______________________________________________________________________
*********************************************************************
**/
#pragma once
#include "bounding_box.h"
#include <opencv2/core.hpp>
class CFTracker {
   public:
    CFTracker(const BoundingBox& init_bbox);

    /**
     * @brief Process the video
     *
     * @param video
     */
    void process(const std::vector<cv::Mat>& video);

   private:
    void preprocessing(cv::Mat& img);
    void train_H(cv::Mat img);
    void update_H(cv::Mat img);
    void update_bbox(cv::Mat img);
    void visualize(cv::Mat img);

    BoundingBox bbox_;

    float rate_ = 0.01f;
    cv::Mat RESPONSE_;
    cv::Mat KERNEL_A_;  //! need to be initialized as 0
    cv::Mat KERNEL_B_;
    cv::Mat KERNEL_;  // H
};