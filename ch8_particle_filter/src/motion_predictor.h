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

class MotionPredictor {
   public:
    /**
     * @brief Construct a new Motion Predictor object
     *
     * @param initial_pos
     */
    MotionPredictor(cv::Point2f initial_pos) : curr_pos_(initial_pos), last_pos_(initial_pos) {
    }
    /**
     * @brief Get the current position
     *
     * @return cv::Point2f
     */
    cv::Point2f get_curr_pos() const {
        return curr_pos_;
    }

    /**
     * @brief Predict motion using last two frames
     *
     * @return cv::Vec2f
     */
    cv::Vec2f predict_motion() const {
        return cv::Point2f(curr_pos_.x - last_pos_.x, curr_pos_.y - last_pos_.y);
    }

    /**
     * @brief Predict position using last two frames
     *
     * @return cv::Point2f
     */
    cv::Point2f predict_positon() const {
        cv::Point2f next_pos(curr_pos_);

        next_pos.x += curr_pos_.x - last_pos_.x;
        next_pos.y += curr_pos_.y - last_pos_.y;
        return next_pos;
    }

    /**
     * @brief Set the position to current position for prediction later
     *
     * @param pos
     */
    void set_observation(cv::Point2f pos) {
        last_pos_ = curr_pos_;
        curr_pos_ = pos;
    }

   private:
    cv::Point2f last_pos_;
    cv::Point2f curr_pos_;
};