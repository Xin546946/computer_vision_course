#pragma once
#include <opencv2/core/core.hpp>

class MeanShiftSeg {
   public:
    MeanShiftSeg(double radius);
    void process(cv::Mat img);

   private:
    void update_mass_center();
    void init_visualization();
    void visualize();

    double radius_square_;

    cv::Mat features_origin_;
    cv::Mat features_curr_;

    // for visualization
    cv::Mat vis_feature_space_;
    cv::Mat vis_img_origin_;
    cv::Mat value_channel_origin_;
};