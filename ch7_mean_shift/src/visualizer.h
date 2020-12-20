#pragma once
#include <array>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <pangolin/pangolin.h>
#include <vector>

class Visualizer {
   public:
    Visualizer();
    void show();
    void set_data(const std::vector<cv::Vec3d>& bgr_datas);
    void shut();

   private:
    void init();
    void draw_points() const;

    // opengl object
    pangolin::View view_3d_;
    pangolin::OpenGlRenderState cam_3d_;

    // Data stored in the order of rgb
    std::mutex points_mutex_;
    std::vector<cv::Vec3d> points_;
};
