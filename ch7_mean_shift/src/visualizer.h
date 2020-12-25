#pragma once
#include <array>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <pangolin/pangolin.h>
#include <vector>

typedef std::pair<cv::Vec3d, cv::Vec3d> line;
class BGR;
class Visualizer {
   public:
    Visualizer() = default;
    void show(int rows, int cols);
    void set_data(const std::vector<cv::Vec3f>& bgr_datas);
    void set_data(const std::vector<BGR>& bgr_datas, const std::vector<cv::Point>& pos);
    void request_shut();

    std::mutex stop_mutex_;
    bool stop_ = true;

   private:
    void init();
    void draw_points();
    void draw_frame() const;
    void show_segmentation(int rows, int cols);

    // opengl object
    pangolin::View view_3d_;
    pangolin::OpenGlRenderState cam_3d_;

    // std::array stored in the order of rgb
    std::mutex points_mutex_;
    std::vector<cv::Vec3f> points_;

    std::vector<cv::Point> pos_;
};

void draw_lines_with_interpolated_color(const std::vector<line>& lines);