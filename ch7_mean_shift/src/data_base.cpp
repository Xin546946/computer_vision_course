#include "data_base.h"
#include "memory"
#include <chrono>
#include <opencv2/core/core.hpp>
#include <thread>

ColorData::ColorData(cv::Mat img, double radius, std::shared_ptr<Visualizer> vis_ptr)
    : r_square_(radius * radius), vis_ptr_(vis_ptr) {
    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            colors_.push_back(img.at<cv::Vec3b>(r, c));
        }
    }

    colors_original_ = colors_;
}

void ColorData::update_mass_center() {
    for (cv::Vec3d& color : colors_) {
        cv::Vec3d acc_rgb(0.0, 0.0, 0.0);
        int num = 0;
        for (const cv::Vec3d& color_original : colors_original_) {
            if (cv::norm(color - color_original, cv::NORM_L2SQR) > r_square_) {
                continue;
            }
            num++;
            acc_rgb += color_original;
        }

        color = acc_rgb / num;
    }
}

void ColorData::visualize() {
    vis_ptr_->set_data(colors_);

    std::unique_lock<std::mutex> ul_stop(vis_ptr_->stop_mutex_);
    while (vis_ptr_->stop_) {
        ul_stop.unlock();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ul_stop.lock();
    }
}
bool ColorData::is_convergent() {
    if (colors_last_.empty()) return false;

    for (int i = 0; i < colors_.size(); i++) {
        if (cv::norm(colors_[i] - colors_last_[i], cv::NORM_L2SQR) > 1e-3) {
            return false;
        }
    }
    return true;
}

void ColorData::back_up_mass_center() {
    colors_last_ = colors_;
}
