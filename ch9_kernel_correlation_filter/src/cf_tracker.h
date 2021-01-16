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
/**
 * @brief Correlation Filter Tracker
 *
 */
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
    /**
     * @brief train init H matrix with random affine transformation
     *
     * @param img
     */
    void train_init_kernel(cv::Mat img);

    /**
     * @brief update H matrix
     *
     * @param img
     */
    void update_kernel(cv::Mat img);

    /**
     * @brief update bounding box
     *
     * @param img
     */
    void update_bbox(cv::Mat img);

    /**
     * @brief visualize the bbox
     *
     * @param img
     */
    void visualize(cv::Mat img);

    BoundingBox bbox_;

    float rate_ = 0.2f;

    cv::Mat RESPONSE_;  // G
    cv::Mat KERNEL_A_;  // A
    cv::Mat KERNEL_B_;  // B
    cv::Mat KERNEL_;    // H
};