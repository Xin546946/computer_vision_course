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
#include <opencv2/core/core.hpp>

typedef cv::Point2f Point;

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

    const Point center() const {
        return Point(window_.tl().x + window_.width / 2, window_.tl().y + window_.height / 2);
    }

    bool contains(Point point) const {
        return window_.contains(point);
    }

   private:
    cv::Rect2f window_;
};
