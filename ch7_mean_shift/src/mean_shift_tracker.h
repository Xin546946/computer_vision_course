#pragma once
#include "mean_shift.h"
#include "tracking_data_base.h"
#include <memory>

class MeanShiftTracker {
   public:
    MeanShiftTracker() = default;
    MeanShiftTracker(std::unique_ptr<TrackerDataBase> db_ptr);

    void process(const std::vector<cv::Mat>& video, const cv::Mat temp);

   private:
    MeanShift ms_;
};