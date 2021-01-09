#include "cf_tracker.h"
#include "bounding_box.h"
#include "opencv_utils.h"

CFTracker::CFTracker(const BoundingBox& bbox) : bbox_(bbox) {
}

void CFTracker::process(const std::vector<cv::Mat>& video) {
    for (cv::Mat frame : video) {
        update_H_conj(H_conj);
    }
}