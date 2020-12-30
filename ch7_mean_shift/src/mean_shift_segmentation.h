#pragma once
#include "visualizer_3d.h"
#include <memory>
#include <opencv2/core/core.hpp>

class MeanShiftSeg {
   public:
    MeanShiftSeg(double radius, Vis3D* vis_ptr);
    void process(cv::Mat img);

   private:
    void update_mass_center();

    double radius_square_;

    cv::Mat features_origin_;
    cv::Mat features_curr_;

    // for visualization
    cv::Mat vis_origin_;
    cv::Mat vis_curr_;
    std::unique_ptr<Vis3D> visualizer;
};