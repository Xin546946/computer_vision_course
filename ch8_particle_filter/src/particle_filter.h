#pragma once
#include "bounding_box.h"
#include "histogram.h"
#include "motion_predictor.h"
#include "opencv2/core.hpp"

class State;
class Particle;

class ParticleFilter {
   public:
    ParticleFilter(cv::Mat temp, const BoundingBox& init_bbox, int num_particles = 100);

    void init_particles(const BoundingBox& init_bbox, int num_particles);
    void update_status();
    void update_weights(cv::Mat frame);
    void resampling();

   private:
    std::vector<Particle> particles_;
    MotionPredictor motion_model_;

    Histogram hist_temp_;
};

struct State {
   public:
    State(float w, float h, float x_center, float y_center);

   private:
    BoundingBox bbox_;
};
class Particle {
   public:
    Particle(float w, float h, float x_center, float y_center, float weight);

   private:
    State state_;
    double weight_;
};
