/**
______________________________________________________________________
*********************************************************************
* @brief  This file is developed for the course of ShenLan XueYuan:
* Fundamental implementations of Computer Vision
* all rights preserved
* @author Xin Jin, Zhaoran Wu
* @contact: xinjin1109@gmail.com, zhaoran.wu1@gmail.com
*
______________________________________________________________________
*********************************************************************
**/
#pragma once
#include "visualizer_3d.h"
#include <memory>
#include <opencv2/core/core.hpp>

class MeanShiftSeg {
   public:
    /**
     * @brief Construct a new Mean Shift Seg object
     *
     * @param [in] radius : the size of the local window(circle)
     * @param [in] vis_ptr: pointer to the visualizer
     */
    MeanShiftSeg(double radius, Vis3D* vis_ptr);
    /**
     * @brief segement the img using mean shift
     *
     * @param [in] img
     */
    void process(cv::Mat img);

   private:
    /**
     * @brief update the position of each data, the new position is the mass center of the local window
     *
     */
    void update_mass_center();

    double radius_square_;  // squre radius

    cv::Mat features_origin_;  // originial input, which will not changed during the segmentation
    cv::Mat features_curr_;    // record the new position of all the features

    std::unique_ptr<Vis3D> visualizer;
};