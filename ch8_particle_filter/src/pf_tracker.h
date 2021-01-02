#pragma once
#include "bounding_box.h"
#include <opencv2/core.hpp>
class PFTracker {
   public:
    PFTracker(cv::Mat temp, const BoundingBox& init_bbox);
    void process(const std::vector<cv::Mat>& video);

   private:
    ParticleFilter pf_;
};
