#include "pf_tracker.h"

PFTracker::PFTracker(cv::Mat temp, const BoundingBox& init_bbox, int num_particles)
    : pf_(temp, init_bbox, num_particles) {
}

void PFTracker::process(const std::vector<cv::Mat>& video) {
    for (cv::Mat frame : video) {
        pf_.update_status();
        pf_.update_weights(frame);
        pf_.resampling();
    }
}
