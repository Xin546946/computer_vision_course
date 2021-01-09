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
    BoundingBox bbox_;
    cv::Mat H_;  // H_ is mask
};