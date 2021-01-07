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
#include "particle_filter.h"
#include <opencv2/core.hpp>
/**
 * @brief Tracking based on Particle Filter
 *
 */
class PFTracker {
   public:
    /**
     * @brief Construct a new PFTracker object
     *
     * @param temp
     * @param init_bbox
     * @param num_particles
     */
    PFTracker(cv::Mat temp, const BoundingBox& init_bbox, int num_particles = 100);
    /**
     * @brief Process the image sequences
     *
     * @param video
     */
    void process(const std::vector<cv::Mat>& video);

   private:
    /**
     * @brief particle filter object to handle with particle filter process
     *
     */
    ParticleFilter pf_;
};
