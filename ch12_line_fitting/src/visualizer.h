#pragma once
#include <opencv2/core.hpp>

class Visualizer {
   public:
    Visualizer(double x_min, double x_max, double y_min, double y_max);

    void add_point(double x, double y, bool is_inlier);
    void add_line(double a, double b, bool draw_thresh_line = false, double threshold = 0.0);
    void show(int delay) const;

   private:
    cv::Mat board_;
    cv::Vec2i up_left_;
};