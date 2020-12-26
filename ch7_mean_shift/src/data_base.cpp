#include "data_base.h"
#include "memory"
#include "visualizer.h"
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
    for (cv::Vec3f& color : colors_) {
        cv::Vec3f acc_rgb(0.0, 0.0, 0.0);
        int num = 0;
        for (const cv::Vec3f& color_original : colors_original_) {
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

BGRData::BGRData(cv::Mat img, double radius, std::shared_ptr<Visualizer> vis_ptr)
    : radius_(radius), vis_ptr_(vis_ptr), quarter_r_square_(radius_ * radius_) {
    int size = img.cols * img.rows;
    colors_.reserve(size);

    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            colors_.emplace_back(img.at<cv::Vec3b>(r, c), r, c);
            traverse_queue_.push(&colors_.back());
        }
    }
    std::cout << traverse_queue_.size() << '\n';
    std::cout << img.rows * img.cols << '\n';

    kdtree_ = new ColorKdTree(&colors_[0], size, 1);
}

/**
 * @brief find mode for one sample
 *
 */
void BGRData::update_mass_center() {
    BGR* ptr_curr = traverse_queue_.front();
    traverse_queue_.pop();
    if (ptr_curr->is_convergent_) return;

    std::unordered_set<BGR*> colors_passed_by;

    int iteration = 0;
    cv::Vec3f color_last = ptr_curr->bgr_ + cv::Vec3f(1.f, 0.f, 0.f);

    while (iteration++ < 50) {
        float diff = cv::norm(color_last, ptr_curr->bgr_, cv::NORM_L2SQR);
        if (diff < 1e-2) break;

        std::vector<BGR*> neighbors = kdtree_->rnn_search(ptr_curr, radius_);

        cv::Vec3f acc_color(0.f, 0.f, 0.f);
        for (auto neigh : neighbors) {
            acc_color += neigh->bgr_;
        }

        color_last = ptr_curr->bgr_;
        ptr_curr->bgr_ = acc_color / static_cast<float>(neighbors.size());

        if (diff > 2) {
            std::for_each(neighbors.begin(), neighbors.end(), [&](auto ptr_neigh) {
                if (ptr_neigh->is_convergent_) {
                    return;
                } else {
                    colors_passed_by.insert(ptr_neigh);
                }
            });
        }
    }
    std::for_each(colors_passed_by.begin(), colors_passed_by.end(), [=](BGR* ptr_pass) {
        if (!ptr_pass->is_convergent_) {
            ptr_pass->is_convergent_ = true;
            ptr_pass->bgr_ = ptr_curr->bgr_;
        }
    });
}

inline bool BGRData::is_convergent() {
    return traverse_queue_.empty();
}

inline void BGRData::back_up_mass_center() {
}

void BGRData::visualize() {
    std::vector<cv::Point> pos;
    for (auto color : colors_) {
        pos.emplace_back(color.col_, color.row_);
    }

    vis_ptr_->set_data(colors_, pos);

    std::unique_lock<std::mutex> ul_stop(vis_ptr_->stop_mutex_);
    while (vis_ptr_->stop_) {
        ul_stop.unlock();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ul_stop.lock();
    }
}
