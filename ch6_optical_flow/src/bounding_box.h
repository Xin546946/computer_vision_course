#pragma once
#include <opencv2/core/core.hpp>

typedef cv::Point Point;

class BoundingBox {
   public:
    BoundingBox(int x, int y, int width, int height) : window_(x, y, width, height){};
    BoundingBox() = default;
    void move(float delta_x, float delta_y) {
        window_.x += delta_x;
        window_.y += delta_y;
    }

    const Point top_left() const {
        return window_.tl();
    }

    const int width() const {
        return window_.width;
    }

    const int height() const {
        return window_.height;
    }

   private:
    cv::Rect2f window_;
};
