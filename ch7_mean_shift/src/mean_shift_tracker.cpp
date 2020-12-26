#include "mean_shift_tracker.h"

MeanShiftTracker::MeanShiftTracker() {
    std::shared_ptr<DataBase> db_ptr = std::make_shared<TrackerDataBase>(cv::Mat(), cv::Mat(), cv::Point2f());
    ms_ = MeanShift(db_ptr);
}

void MeanShiftTracker::process(const std::vector<cv::Mat>& video, const cv::Mat temp) {
}