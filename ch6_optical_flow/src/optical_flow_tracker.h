#pragma once

#include <opencv2/core/core.hpp>

class OpticalFlowTracker {
   public:
    OpticalFlowTracker() = default;

    void process(BoundingBox initial_bbox, const std::vector<cv::Mat>& video);

   private:
    FeaturePointsManager feature_points_manager_;
};