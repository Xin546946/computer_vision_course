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
#include "feature_points_manager.h"
#include <opencv2/core/core.hpp>
/**
 * @brief class for tracking algorithm based on optical flow method
 *
 */
class OpticalFlowTracker {
   public:
    /**
     * @brief Construct a new Optical Flow Tracker object
     *
     */
    OpticalFlowTracker() = default;
    /**
     * @brief Process the videos (image sequences) with an initial bounding box, and videos
     *
     * @param initial_bbox
     * @param video
     */
    void process(BoundingBox initial_bbox, const std::vector<cv::Mat>& video);

   private:
    /**
     * @brief Manage feature points
     *
     */
    FeaturePointsManager feature_points_manager_;
};