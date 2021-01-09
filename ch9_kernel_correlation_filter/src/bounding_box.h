/**
______________________________________________________________________
*********************************************************************
* @brief This file is developed for the course of ShenLan XueYuan:
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
class BoundingBox {
   public:
    BoundingBox(float x, float y, float width, float height) : window_(x, y, width, height){};
    BoundingBox() = default;
    /**
     * @brief move a bounding box with a given delta motion
     *
     * @param [in] delta_x
     * @param [in] delta_y
     */
    void move(float delta_x, float delta_y) {
        window_.x += delta_x;
        window_.y += delta_y;
    }

    /**
     * @brief move the bbox to a new position, which is specified with top left corner
     *
     * @param [in] x
     * @param [in] y
     */
    void move_top_left_to(float x, float y) {
        window_.x = x;
        window_.y = y;
    }

    /**
     * @brief move the bbox to a new position, which is specified with center position
     *
     * @param [in] x
     * @param [in] y
     */
    void move_center_to(float x, float y) {
        return move_top_left_to(x - window_.width / 2.f, y - window_.height / 2.f);
    }

    const cv::Point2f top_left() const {
        return window_.tl();
    }
    const cv::Point2f bottom_right() const {
        return window_.br();
    }

    int area() const {
        return window_.width * window_.height;
    }

    int width() const {
        return window_.width;
    }

    int height() const {
        return window_.height;
    }

    cv::Size2f size() const {
        return window_.size();
    }

    void resize(float w, float h) {
        window_.width = w;
        window_.height = h;
    }

    const cv::Point2f center() const {
        return cv::Point2f(window_.tl().x + window_.width / 2, window_.tl().y + window_.height / 2);
    }

    bool contains(cv::Point2f point) const {
        return window_.contains(point);
    }

    BoundingBox set_larger_roi(double ratio_width, double ratio_height) {
        return BoundingBox(this->center().x - this->width() * ratio_width / 2,
                           this->center().y - this->height() * ratio_height / 2, ratio_width * this->width(),
                           ratio_height * this->height());
    }

   private:
    cv::Rect2f window_;
};
