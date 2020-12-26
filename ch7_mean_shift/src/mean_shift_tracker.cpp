#include "mean_shift_tracker.h"

MeanShiftTracker::MeanShiftTracker() {
    TrackerDataBase* db_raw_ptr = new TrackerDataBase(cv::Mat(), cv::Mat(), cv::Point2f());
    db_ptr_ = std::shared_ptr<TrackerDataBase>(db_raw_ptr);
    ms_ = MeanShift(std::shared_ptr<DataBase>(db_raw_ptr));
}

void MeanShiftTracker::process(const std::vector<cv::Mat>& video, const cv::Mat temp) {
    db_ptr_.temp_ = temp;

    while ((/* condition */)) {
        db_ptr_->initial_pos = motion_predictor.next_pos();

        ms_.run();

        cv::Point2f new_pos = db_ptr_->get_tracking_result();

        motion_predictor.set_tracking_result(new_pos);
        /* code */
    }
}