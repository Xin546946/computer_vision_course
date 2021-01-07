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
#include "bounding_box.h"
#include "histogram.h"
#include "motion_predictor.h"
#include "opencv2/core.hpp"
#include <array>

class State;
struct Particle;

class ParticleFilter {
   public:
    /**
     * @brief Construct a new Particle Filter object
     *
     * @param temp
     * @param init_bbox
     * @param num_particles
     * @param num_histogramm_bins
     */
    ParticleFilter(cv::Mat temp, const BoundingBox& init_bbox, int num_particles = 100, int num_histogramm_bins = 256);
    /**
     * @brief Initialize particles
     *
     * @param init_bbox
     * @param num_particles
     */
    void init_particles(const BoundingBox& init_bbox, int num_particles);

    /**
     * @brief Predict states
     *
     */
    void predict_status();

    /**
     * @brief Update weights
     *
     * @param frame
     */
    void update_weights(cv::Mat frame);

    /**
     * @brief Resampling
     *
     */
    void resampling();

    /**
     * @brief Compute mean state and set observation
     *
     * @return State
     */
    State compute_mean_state_and_set_observation();

    /**
     * @brief Visualize particles on the frame
     *
     * @param frame
     */
    void visualize(cv::Mat frame);

   private:
    std::vector<Particle> particles_;
    MotionPredictor motion_model_;  // assume a constant velocity model

    Histogram hist_temp_;
};

class State {
   public:
    /**
     * @brief Construct a new State object
     *
     * @param w
     * @param h
     * @param x_center
     * @param y_center
     */
    State(float w, float h, float x_center, float y_center);

    float w() const {
        return bbox_.width();
    }
    float h() const {
        return bbox_.height();
    }

    float x_center() const {
        return bbox_.center().x;
    }

    float y_center() const {
        return bbox_.center().y;
    }

    cv::Point2f center() const {
        return bbox_.center();
    }

    void resize(float w, float h) {
        bbox_.resize(w, h);
    }
    cv::Size2f size() {
        return bbox_.size();
    }
    void move(float delta_x, float delta_y) {
        bbox_.move(delta_x, delta_y);
    }

   private:
    BoundingBox bbox_;
};
struct Particle {
    /**
     * @brief Construct a new Particle object
     *
     * @param w
     * @param h
     * @param x_center
     * @param y_center
     * @param weight
     */
    Particle(float w, float h, float x_center, float y_center, float weight);

    /**
     * @brief Construct a new Particle object
     *
     * @param state
     * @param weight
     */
    Particle(State state, double weight);

    /**
     * @brief Update particles with motion and noise
     *
     * @param delta_motion
     * @param noise
     */
    void update_with_motion_and_noise(cv::Vec2f delta_motion, const std::array<double, 4>& noise);

    bool bad_ = false;
    State state_;
    double weight_;
};
