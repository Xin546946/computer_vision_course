#pragma once
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/viz.hpp>

class Vis3D {
   public:
    Vis3D();
    void set_features(cv::Mat points_mat);
    void visualize();

   private:
    cv::viz::Viz3d window_;

    cv::viz::WArrow x_axis_;
    cv::viz::WArrow y_axis_;
    cv::viz::WArrow z_axis_;

    cv::viz::WCube cube_;

    std::mutex mutex_features_;
    std::shared_ptr<cv::viz::WCloud> ptr_features_ = nullptr;
};