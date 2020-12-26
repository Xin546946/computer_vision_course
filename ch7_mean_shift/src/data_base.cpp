#include "data_base.h"
#include "memory"
#include "tictoc.h"
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>

BGR operator+(const BGR& lhs, const BGR& rhs) {
    return BGR(lhs.bgr_ + rhs.bgr_);
}

inline std::array<int, 3> vec2array(const cv::Vec3b& data) {
    return {data[0], data[1], data[2]};
}

inline cv::Vec3b array2vec(const std::array<int, 3> data) {
    return cv::Vec3b(data[0], data[1], data[2]);
}

ColorData::ColorData(cv::Mat img, double radius, std::shared_ptr<Visualizer> vis_ptr)
    : r_square_(radius * radius), vis_ptr_(vis_ptr) {
    std::vector<std::array<int, 3>> kdtree_img;
    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            colors_.push_back(img.at<cv::Vec3b>(r, c));
            kdtree_img.push_back(vec2array(img.at<cv::Vec3b>(r, c)));
        }
    }
    kdtree_ = new KDTree3D(kdtree_img, 2);
    std::cout << "Finish to build KDTree! " << '\n';
}

void ColorData::update_mass_center() {
    for (cv::Vec3f& color : colors_) {
        cv::Vec3f acc_rgb(0.0, 0.0, 0.0);
        int num = 0;
        for (const cv::Vec3f& color2 : colors_) {
            if (cv::norm(color - color2, cv::NORM_L2SQR) > r_square_) {
                continue;
            }
            num++;
            acc_rgb += color2;
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
    : radius_(radius), vis_ptr_(vis_ptr), quarter_r_square_(0.25 * radius_ * radius_) {
    int size = img.cols * img.rows;
    colors_.reserve(size);

    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            colors_.emplace_back(img.at<cv::Vec3b>(r, c));
            traverse_queue_.push(&colors_.back());
        }
    }
    std::cout << traverse_queue_.size() << '\n';
    std::cout << img.rows * img.cols << '\n';

    kdtree_ = new ColorKdTree(&colors_[0], size, 1);
}

inline float compute_square_dist(const BGR& lhs, const BGR& rhs) {
    return cv::norm(lhs.bgr_ - rhs.bgr_, cv::NORM_L2SQR);
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
    std::vector<BGR*> neighbors;

    int iteration = 0;
    BGR color_last = *ptr_curr + BGR(1, 0, 0);

    while (iteration++ < 50 && compute_square_dist(color_last, *ptr_curr) > 1e-4) {
        neighbors = kdtree_->rnn_search(ptr_curr, radius_);
        std::cout << "kd tree rnn num : " << neighbors.size() << '\n';
        std::for_each(neighbors.begin(), neighbors.end(), [&](auto ptr_neigh) {
            if (ptr_neigh->is_convergent_) {
                return;
            } else if (BGR::is_in_radius(ptr_curr, ptr_neigh, quarter_r_square_)) {
                std::cout << "ptr_negh should be inserted." << '\n';
                colors_passed_by.insert(ptr_neigh);
            }
        });
        std::cout << "num of pass by :" << colors_passed_by.size() << '\n';
        cv::Vec3f acc_color(0.f, 0.f, 0.f);
        for (auto neigh : neighbors) {
            acc_color += neigh->bgr_;
        }

        color_last = *ptr_curr;
        ptr_curr->bgr_ = acc_color / static_cast<float>(neighbors.size());
    }
    std::cout << "num of pass by :" << colors_passed_by.size() << '\n';
    std::for_each(colors_passed_by.begin(), colors_passed_by.end(), [=](BGR* ptr_pass) {
        if (!ptr_pass->is_convergent_) {
            ptr_pass->is_convergent_ = true;
            ptr_pass->bgr_ = ptr_curr->bgr_;
        }
    });

    std::cout << "num of mode neighbors :" << neighbors.size() << '\n';
    std::for_each(neighbors.begin(), neighbors.end(), [=](BGR* ptr_pass) {
        if (!ptr_pass->is_convergent_) {
            ptr_pass->is_convergent_ = true;
            ptr_pass->bgr_ = ptr_curr->bgr_;
        }
    });
}

bool BGRData::is_convergent() {
    return traverse_queue_.empty();
}

void BGRData::back_up_mass_center() {
}

void BGRData::visualize() {
    vis_ptr_->set_data(colors_);

    std::unique_lock<std::mutex> ul_stop(vis_ptr_->stop_mutex_);
    while (vis_ptr_->stop_) {
        ul_stop.unlock();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ul_stop.lock();
    }
}
