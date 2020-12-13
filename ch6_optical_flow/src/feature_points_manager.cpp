#include "feature_points_manager.h"
#include "opencv_utils.h"
#include <array>
#include <numeric>
#include <opencv2/video/tracking.hpp>

std::vector<cv::Point2f> extract_feature_points(cv::Mat img, cv::Mat mask, float weight);
std::vector<cv::Point2f> process_feature_points(std::vector<cv::Point2f> feature_points, std::vector<uchar> status);
// std::array<float, 2> compute_incremental_move(std::vector<cv::Point2f> feature_points);
float median(std::vector<float> data);
// FeaturePointsManager::FeaturePointsManager() {
// }

std::vector<cv::Point2f>& operator+=(std::vector<cv::Point2f>& feature_points_1,
                                     std::vector<cv::Point2f>& feature_points_2) {
    for (cv::Point2f point : feature_points_2) {
        feature_points_1.push_back(point);
    }
    return feature_points_1;
}

void FeaturePointsManager::initialize(cv::Mat img, BoundingBox initial_bbox) {
    bbox_ = initial_bbox;
    extract_new_feature_points(img);
}

void FeaturePointsManager::extract_new_feature_points(cv::Mat img) {
    cv::Mat mask = compute_mask(img.rows, img.cols);
    int iter = 0;
    float weight = 1.0f;
    while (!is_enough_points() && iter < 4) {
        iter++;
        std::vector<cv::Point2f> additional_feature_points = extract_feature_points(img, mask, weight);
        feature_points_ += additional_feature_points;
        for (cv::Point2f point : additional_feature_points) {
            put_val_around(0, mask, point.x, point.y, 3, 3);
        }
        weight *= 0.6;
    }
}

std::vector<cv::Point2f> extract_feature_points(cv::Mat img, cv::Mat mask, float weight) {
    std::vector<cv::Point2f> feature_points;
    cv::goodFeaturesToTrack(img, feature_points, 200, weight * 0.1, weight * 10, mask);
    return feature_points;
}

cv::Mat FeaturePointsManager::compute_mask(int rows, int cols) {
    cv::Mat mask = cv::Mat::zeros(cv::Size(cols, rows), CV_8UC1);
    put_val_from_ul(255, mask, bbox_.top_left().x, bbox_.top_left().y, cols, rows);
    for (cv::Point2f point : feature_points_) {
        put_val_around(0, mask, point.x, point.y, 3, 3);
    }
    return mask;
}

// //! 这个函数的意义?
// cv::Mat FeaturePointsManager::compute_curr_mask(cv::Mat img, BoundingBox bbox) {
//     cv::Mat mask = cv::Mat::zeros(img.size(), img.type());
//     cv::Rect mask_rect = cv::Rect(cv::Point(0, 0), mask.size());
//     cv::Rect bbox_rect = cv::Rect(bbox.top_left(), cv::Size(bbox.width(), bbox.height()));
//     cv::Rect intersection = mask_rect & bbox_rect;
//     mask(intersection) = cv::Mat::ones(cv::Size(bbox.width(), bbox.height()), img.type());
//     return mask;
// }

// void FeaturePointsManager::process_num_feature_points() {
//     int max_iter = 10;
//     int iter = 0;
//     while (!is_enough() && iter < max_iter) {
//         iter++;
//         // todo overload +=
//         feature_points_ += get_feature_points();  //! ??????
//     }
//     if (!is_enough()) {
//         // todo
//     }
// }

// void FeaturePointsManager::set_tracking_results(const std::vector<cv::Point2f>& tracked_feature_points,
//                                                 const std::vector<uchar>& status) {
//     delete_bad_feature_points(tracked_feature_points, status);
//     update_bbox();
// }

// void FeaturePointsManager::delete_bad_feature_points(const std::vector<cv::Point2f>& tracked_feature_points,
//                                                      const std::vector<uchar>& status) {
//     // todo calc_with_direction, then delete_with_direction
//     std::vector<float> amplitude = compute_with_amplitude(tracked_feature_points);
//     std::vector<float> angle = compute_with_direction(tracked_feature_points);
//     delete_with_direction(amplitude);
//     delete_with_amplitude(angle);
//     delete_with_status(status);
// }

// void FeaturePointsManager::delete_with_status(const std::vector<uchar>& status) {
//     for (int i = 0; i < status.size(); i++) {
//         if (int(status[i]) == 0) {
//             feature_points_.erase(feature_points_.begin() + i);
//         }
//     }
// }

// std::vector<float>& FeaturePointsManager::compute_with_direction(
//     const std::vector<cv::Point2f>& tracked_feature_points) {
//     std::vector<float> angle;
//     for (int i = 0; i < tracked_feature_points.size(); i++) {
//         angle.emplace_back((tracked_feature_points[i].y - feature_points_[i].y) /
//                            (tracked_feature_points[i].x - feature_points_[i].x));
//     }
//     return angle;
// }

// std::vector<float>& FeaturePointsManager::compute_with_amplitude(
//     const std::vector<cv::Point2f>& tracked_feature_points) {
//     std::vector<float> amplitude;
//     for (int i = 0; i < tracked_feature_points.size(); i++) {
//         amplitude.emplace_back(sqrt(pow(tracked_feature_points[i].y - feature_points_[i].y, 2) +
//                                     pow(tracked_feature_points[i].x - feature_points_[i].x, 2)));
//     }
//     return amplitude;
// }

// void FeaturePointsManager::delete_with_amplitude(const std::vector<float>& amplitude) {
//     float sum_of_elems = std::accumulate(amplitude.begin(), amplitude.end(), 0);
//     // calculate variance
// }

// void FeaturePointsManager::delete_with_direction(const std::vector<float>& angle) {
// }

void FeaturePointsManager::process_feature_points(cv::Mat img, const std::vector<cv::Point2f>& new_feature_points,
                                                  std::vector<uchar>& status) {
    vis_optical_flow(img, this->feature_points_, new_feature_points);
    update_status(new_feature_points, status);
    update_bbox(new_feature_points, status);
    update_feature_points(new_feature_points, status);
    cv::Mat mask = compute_mask(img.rows, img.cols);
    extract_new_feature_points(img);
    // adjust_bbox();
}

void FeaturePointsManager::update_bbox(const std::vector<cv::Point2f>& new_feature_points, std::vector<uchar>& status) {
    // todo rewrite!
    float delta_x = 0.0f;
    float delta_y = 0.0f;
    for (cv::Point2f point : feature_points_) {
        delta_x += point.x;
        delta_y += point.y;
    }
    delta_x /= feature_points_.size();
    delta_y /= feature_points_.size();
    bbox_.move(delta_x, delta_y);
}

// void FeaturePointsManager::adjust_bbox() {
//     std::array<float, 2> delta = compute_incremental_move(this->feature_points_);
//     update_bbox(delta[0], delta[1]);
// }

// std::array<float, 2> compute_incremental_move(std::vector<cv::Point2f> feature_points) {
//     std::array<float, 2> delta{0.0f, 0.0f};
//     for (cv::Point2f point : feature_points) {
//         delta[0] += point.x;
//         delta[1] += point.y;
//     }
//     return delta;
// }

void FeaturePointsManager::update_status(const std::vector<cv::Point2f>& new_feature_points,
                                         std::vector<uchar>& status) {
    std::vector<cv::Vec2f> motion = compute_pixel_motion(this->feature_points_, new_feature_points, status);
    mark_status_with_amplitude(motion, status);
    mark_status_with_angle(motion, status);
}

void FeaturePointsManager::update_feature_points(const std::vector<cv::Point2f>& new_feature_points,
                                                 std::vector<uchar>& status) {
    // todo
}

void FeaturePointsManager::mark_status_with_amplitude(const std::vector<cv::Vec2f>& motion,
                                                      std::vector<uchar>& status) {
    // todo
}
void FeaturePointsManager::mark_status_with_angle(const std::vector<cv::Vec2f>& motion, std::vector<uchar>& status) {
    // todo
}
std::vector<cv::Vec2f> compute_pixel_motion(const std::vector<cv::Point2f>& feature_points_,
                                            const std::vector<cv::Point2f>& new_feature_points) {
    // todo
}

float median(std::vector<float> data) {
    // todo
}

void vis_optical_flow(cv::Mat img, const std::vector<cv::Point2f>& feature_points_,
                      const std::vector<cv::Point2f>& new_feature_points) {
    // todo
}