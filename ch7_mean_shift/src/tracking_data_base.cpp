#include "tracking_data_base.h"
#include "data_base.h"
#include "memory"
#include "opencv_utils.h"
#include "visualizer.h"
#include <chrono>
#include <opencv2/core/core.hpp>
#include <thread>

TrackerDataBase::TrackerDataBase(cv::Mat img, cv::Mat temp, cv::Point2f initial_pos)
    : img_(img),
      temp_(temp),
      pos_(initial_pos),
      bbox_(pos_.x - temp_.cols / 2.0f, pos_.y - temp_.rows / 2.0f, static_cast<float>(temp_.cols),
            static_cast<float>(temp_.rows)) {
}

cv::Point2f TrackerDataBase::get_tracking_result() const {
    return pos_;
}

void TrackerDataBase::init_mass_center() {
    if (initialized_ == false) {
        // template matching and get the bbox
        cv::Point2i center = template_matching(img_, temp_);
        initialized_ = true;
    } else {
        // predict the initial pos for tracking at the next frame
    }
}