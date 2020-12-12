#include "feature_points_manager.h"
#include <numeric>
#include <opencv2/video/tracking.hpp>

// todo constructor
FeaturePointsManager::FeaturePointsManager() {
}

std::vector<cv::Point2f>& FeaturePointsManager::operator+=(const std::vector<cv::Point2f>& feature_points) {
    for (cv::Point2f point : feature_points) {
        this->feature_points_.emplace_back(point);
        return this->feature_points_;
    }
}

void FeaturePointsManager::initialize(cv::Mat img, BoundingBox initial_bbox) {
    set_bounding_box(initial_bbox);
    extract_new_feature_points(img);
}

void FeaturePointsManager::extract_new_feature_points(cv::Mat img) {
    cv::Mat mask = compute_curr_mask(img);

    //! 分两步, 还是一步?
    extract_feature_points(img, mask);
    process_num_feature_points();
}

void FeaturePointsManager::extract_feature_points(cv::Mat img, cv::Mat mask) {
    //! 新提取的特征点不应该直接覆盖,而是加上
    vector<Point2f> fps;
    cv::goodFeaturesToTrack(img, fps, 200, 0.01, 10, mask);
    feature_points_ += fps;
}

cv::Mat FeaturePointsManager::compute_curr_mask(cv::Mat img) {
    //! image 参数的意义?
    //! 已经存在的特征点没有参与其中
    cv::Mat mask = cv::Mat::zeros(img.size(), img.type());
    cv::Rect mask_rect = cv::Rect(cv::Point(0, 0), mask.size());
    cv::Rect bbox_rect = cv::Rect(bbox_.top_left(), cv::Size(bbox_.width(), bbox_.height()));
    cv::Rect intersection = mask_rect & bbox_rect;
    mask(intersection) = cv::Mat::ones(cv::Size(bbox_.width(), bbox_.height()), img.type());
    return mask;
}

//! 这个函数的意义?
cv::Mat FeaturePointsManager::compute_curr_mask(cv::Mat img, BoundingBox bbox) {
    cv::Mat mask = cv::Mat::zeros(img.size(), img.type());
    cv::Rect mask_rect = cv::Rect(cv::Point(0, 0), mask.size());
    cv::Rect bbox_rect = cv::Rect(bbox.top_left(), cv::Size(bbox.width(), bbox.height()));
    cv::Rect intersection = mask_rect & bbox_rect;
    mask(intersection) = cv::Mat::ones(cv::Size(bbox.width(), bbox.height()), img.type());
    return mask;
}

void FeaturePointsManager::process_num_feature_points() {
    int max_iter = 10;
    int iter = 0;
    while (!is_enough() && iter < max_iter) {
        iter++;
        // todo overload +=
        feature_points_ += get_feature_points();  //! ??????
    }
    if (!is_enough()) {
        // todo
    }
}

void FeaturePointsManager::set_tracking_results(const std::vector<cv::Point2f>& tracked_feature_points,
                                                const std::vector<uchar>& status) {
    delete_bad_feature_points(tracked_feature_points, status);
    update_bbox();
}

void FeaturePointsManager::delete_bad_feature_points(const std::vector<cv::Point2f>& tracked_feature_points,
                                                     const std::vector<uchar>& status) {
    // todo calc_with_direction, then delete_with_direction
    std::vector<float> amplitude = compute_with_amplitude(tracked_feature_points);
    std::vector<float> angle = compute_with_direction(tracked_feature_points);
    delete_with_direction(amplitude);
    delete_with_amplitude(angle);
    delete_with_status(status);
}

void FeaturePointsManager::delete_with_status(const std::vector<uchar>& status) {
    for (int i = 0; i < status.size(); i++) {
        if (int(status[i]) == 0) {
            feature_points_.erase(feature_points_.begin() + i);
        }
    }
}

std::vector<float>& FeaturePointsManager::compute_with_direction(
    const std::vector<cv::Point2f>& tracked_feature_points) {
    std::vector<float> angle;
    for (int i = 0; i < tracked_feature_points.size(); i++) {
        angle.emplace_back((tracked_feature_points[i].y - feature_points_[i].y) /
                           (tracked_feature_points[i].x - feature_points_[i].x));
    }
    return angle;
}

std::vector<float>& FeaturePointsManager::compute_with_amplitude(
    const std::vector<cv::Point2f>& tracked_feature_points) {
    std::vector<float> amplitude;
    for (int i = 0; i < tracked_feature_points.size(); i++) {
        amplitude.emplace_back(sqrt(pow(tracked_feature_points[i].y - feature_points_[i].y, 2) +
                                    pow(tracked_feature_points[i].x - feature_points_[i].x, 2)));
    }
    return amplitude;
}

void FeaturePointsManager::delete_with_amplitude(const std::vector<float>& amplitude) {
    float sum_of_elems = std::accumulate(amplitude.begin(), amplitude.end(), 0);
    // calculate variance
}

void FeaturePointsManager::delete_with_direction(const std::vector<float>& angle) {
}

void FeaturePointsManager::update_bbox() {
    float delta_x = 0;
    float delta_y = 0;
    for (cv::Point2f point : feature_points_) {
        delta_x += point.x;
        delta_y += point.y;
    }
    delta_x /= feature_points_.size();
    delta_y /= feature_points_.size();
    bbox_.move(delta_x, delta_y);
}