#include "pf_tracker.h"
#include "bounding_box.h"
#include "opencv_utils.h"

PFTracker::PFTracker(cv::Mat temp, const BoundingBox& init_bbox, int num_particles)
    : pf_(temp, init_bbox, num_particles) {
}

void PFTracker::process(const std::vector<cv::Mat>& video) {
    for (cv::Mat frame : video) {
        State mean_state = pf_.compute_mean_state_and_set_observation();

        cv::Mat vis;
        cv::cvtColor(frame, vis, CV_GRAY2BGR);
        draw_bounding_box_vis_image(vis, mean_state.x_center() - mean_state.w() / 2,
                                    mean_state.y_center() - mean_state.h() / 2, mean_state.w(), mean_state.h());
        cv::imshow("particle filter tracking", vis);
        cv::waitKey(1);

        pf_.visualize(frame);
        pf_.predict_status();
        pf_.visualize(frame);
        pf_.update_weights(frame);
        // todo check if need resampling
        pf_.resampling();
        pf_.visualize(frame);
    }
}
