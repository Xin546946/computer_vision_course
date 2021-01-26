#pragma once
#include <opencv2/core.hpp>

class Visualizer {
   public:
    Visualizer(int height, int width);
    void add_point(const cv::Point2d& point, bool is_inlier);
    void add_line(double a, double b, bool draw_thresh_line = false, double threshold = 0.0);
    void show(int delay) const;

   private:
    cv::Mat board_;
};