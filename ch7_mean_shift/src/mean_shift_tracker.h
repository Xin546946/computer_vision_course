#pragma once
#include "bounding_box.h"
#include "mean_shift.h"
#include "tracking_data_base.h"
#include <memory>

class MeanShiftTracker {
   public:
    MeanShiftTracker();

    void process(const std::vector<cv::Mat>& video, const cv::Mat temp);

   private:
    cv::Mat temp_;
    MeanShift ms_;
    std::shared_ptr<TrackerDataBase> db_ptr_;
};