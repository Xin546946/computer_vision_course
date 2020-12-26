#pragma once
#include <opencv2/core/core.hpp>

class MotionPredictor {
   public:
    MotionPredictor(cv::Point2f initial_pos) : curr_pos_(initial_pos) {
        back_up();
    }

    void back_up() {
        last_pos_ = curr_pos_;
    }

    cv::Point2f next_pos() {
        curr_pos_.x = 2 * curr_pos_.x - last_pos_.x;
        curr_pos_.y = 2 * curr_pos_.y - last_pos_.y;
        return curr_pos_;
    }

    void set_tracking_result(cv::Point2f pos) {
        back_up();
        curr_pos_ = pos;
    }

   private:
    cv::Point2f last_pos_;
    cv::Point2f curr_pos_;
};