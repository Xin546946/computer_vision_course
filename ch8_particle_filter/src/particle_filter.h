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
     * @param temp : template image
     * @param init_bbox
     * @param num_particles
     * @param num_histogramm_bins : number of histogram bins
     */
    ParticleFilter(cv::Mat temp, const BoundingBox& init_bbox, int num_particles = 100, int num_histogramm_bins = 256);

    /**
     * @brief initialize particles w.r.t. to the bounding box
     *
     * @param init_bbox
     * @param num_particles
     */
    void init_particles(const BoundingBox& init_bbox, int num_particles);

    /**
     * @brief predict states, which are property of bounding box
     *
     */
    void predict_status();

    /**
     * @brief update weight of particles
     *
     * @param frame
     */
    void update_weights(cv::Mat frame);

    /**
     * @brief resample the particles
     *
     */
    void resampling();

    /**
     * @brief compute mean state of the particles and set it to motion model for prediction
     *
     * @return State
     */
    State compute_mean_state_and_set_observation();
    void visualize(cv::Mat frame);

   private:
    std::vector<Particle> particles_;
    MotionPredictor motion_model_;  // assume a constant velocity model

    Histogram hist_temp_;
};

class State {
   public:
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
    Particle(float w, float h, float x_center, float y_center, float weight);
    Particle(State state, double weight);

    void update_with_motion_and_noise(cv::Vec2f delta_motion, const std::array<double, 4>& noise);

    bool bad_ = false;
    State state_;
    double weight_;
};
