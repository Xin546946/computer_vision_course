#pragma once
#include "bounding_box.h"
#include "histogram.h"
#include "motion_predictor.h"
#include "opencv2/core.hpp"
#include <array>

class State;
class Particle;

class ParticleFilter {
   public:
    ParticleFilter(cv::Mat temp, const BoundingBox& init_bbox, int num_particles = 100, int num_histogramm_bins = 16);

    void init_particles(const BoundingBox& init_bbox, int num_particles);
    void update_status();
    void update_weights(cv::Mat frame);
    void resampling();
    State compute_mean_state();

   private:
    std::vector<Particle> particles_;
    MotionPredictor motion_model_;  // assume a constant velocity model

    Histogram hist_temp_;
};

struct State {
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

    void update_with_motion_and_noise(cv::Vec2f delta_motion, std::array<double, 4> noise);

    State state_;
    double weight_;
};
