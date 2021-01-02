#pragma once
#include "particle_filter.h"
#include <opencv2/core.hpp>
class PFTracker {
   public:
    PFTracker(cv::Mat temp, const BoundingBox& init_bbox, int num_particles = 100);
    void process(const std::vector<cv::Mat>& video);

   private:
    ParticleFilter pf_;
};
