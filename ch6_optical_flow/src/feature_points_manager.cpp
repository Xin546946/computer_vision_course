#include "feature_points_manager.h"
#include "math_utils.h"
#include "opencv_utils.h"
#include <algorithm>
#include <array>
#include <iostream>
#include <numeric>
#include <opencv2/video/tracking.hpp>

std::vector<cv::Point2f> extract_feature_points(cv::Mat img, cv::Mat mask, float weight);
std::vector<cv::Point2f> process_feature_points(std::vector<cv::Point2f> feature_points, std::vector<uchar> status);
// std::array<float, 2> compute_incremental_move(std::vector<cv::Point2f> feature_points);
float median(std::vector<float> data);
float median(std::vector<float> data, const std::vector<uchar>& status);
std::vector<cv::Vec2f> compute_pixel_motion(const std::vector<cv::Point2f>& old_feature_points,
                                            const std::vector<cv::Point2f>& new_feature_points);
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
        std::cout << "add new feature points, num : " << additional_feature_points.size() << '\n';

        feature_points_ += additional_feature_points;
        for (cv::Point2f point : additional_feature_points) {
            put_val_around(0, mask, point.x, point.y, 3, 3);
        }

        weight *= 0.8;
    }
}

std::vector<cv::Point2f> extract_feature_points(cv::Mat img, cv::Mat mask, float weight) {
    std::vector<cv::Point2f> feature_points;

    double quality_level = std::max(0.02, weight * 0.5);
    double min_distance = std::max(2.0f, weight * 8);

    cv::goodFeaturesToTrack(img, feature_points, 50, quality_level, min_distance, mask);
    return feature_points;
}

cv::Mat FeaturePointsManager::compute_mask(int rows, int cols) {
    cv::Mat mask = cv::Mat::zeros(cv::Size(cols, rows), CV_8UC1);

    put_val_from_ul(255, mask, bbox_.top_left().x, bbox_.top_left().y, bbox_.width(), bbox_.height());
    for (cv::Point2f point : feature_points_) {
        put_val_around(0, mask, point.x, point.y, 3, 3);
    }
    return mask;
}

void FeaturePointsManager::process_feature_points(cv::Mat img,
                                                  const std::vector<cv::Point2f>& feature_points_at_new_position,
                                                  std::vector<uchar>& status) {
    visualize(img, feature_points_at_new_position);

    std::vector<cv::Vec2f> motion = compute_pixel_motion(this->feature_points_, feature_points_at_new_position);
    update_status(motion, status);
    update_bbox(motion, status);
    update_feature_points(feature_points_at_new_position, status);
    cv::Mat mask = compute_mask(img.rows, img.cols);
    extract_new_feature_points(img);
    // static int i = 0;
    // std::cout << "curr img id :" << i++ << " num fps : " << feature_points_.size() << " window tl :" <<
    // bbox_.top_left()
    //          << '\n';

    // adjust_bbox();
}

void FeaturePointsManager::visualize(cv::Mat img, const std::vector<cv::Point2f>& feature_points_at_new_position) {
    cv::Mat vis;
    cv::cvtColor(img, vis, cv::COLOR_GRAY2BGR);

    draw_points(vis, feature_points_, cv::Scalar(255, 0, 0));
    draw_arrowed_lines(vis, feature_points_, feature_points_at_new_position, cv::Scalar(0, 0, 255), 1);

    auto tl = bbox_.top_left();
    draw_bounding_box_vis_image(vis, tl.x - 0.05 * bbox_.width(), tl.y - 0.05 * bbox_.height(), 1.1f * bbox_.width(),
                                1.1f * bbox_.height());  // todo

    cv::imshow("Optical flow tracker", vis);
    cv::waitKey(1);
}

void FeaturePointsManager::update_bbox(const std::vector<cv::Vec2f>& motions, std::vector<uchar>& status) {
    // todo rewrite!
    cv::Vec2f delta_motion;

    cv::Vec2f acc_motion;
    int num_valid = 0;
    for (int i = 0; i < motions.size(); i++) {
        if (status[i]) {
            acc_motion += motions[i];
            num_valid++;
        }
    }
    delta_motion[0] = acc_motion[0] / num_valid;
    delta_motion[1] = acc_motion[1] / num_valid;

    std::cout << " delete : " << status.size() - num_valid << " preserve : " << num_valid << '\n';
    bbox_.move(delta_motion[0], delta_motion[1]);
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

void FeaturePointsManager::update_status(const std::vector<cv::Vec2f>& motion, std::vector<uchar>& status) {
    mark_status_with_contained_points(status);
    mark_status_with_amplitude(motion, status, 1.2);
    mark_status_with_angle(motion, status, 25);
}

void FeaturePointsManager::update_feature_points(const std::vector<cv::Point2f>& feature_points_at_new_position,
                                                 std::vector<uchar>& status) {
    assert(feature_points_at_new_position.size() == status.size());

    feature_points_.clear();

    auto it_status = status.begin();
    std::copy_if(feature_points_at_new_position.begin(), feature_points_at_new_position.end(),
                 std::back_inserter(feature_points_), [&](cv::Point2f) { return *it_status++; });
}

void FeaturePointsManager::mark_status_with_contained_points(std::vector<uchar>& status) {
    auto it = feature_points_.begin();
    std::replace_if(status.begin(), status.end(), [&](uchar s) { return !bbox_.contains(*it++); }, 0);
}
void FeaturePointsManager::mark_status_with_amplitude(const std::vector<cv::Vec2f>& motion, std::vector<uchar>& status,
                                                      float ratio) {
    std::vector<float> amplitude_vec(motion.size());
    std::transform(motion.begin(), motion.end(), amplitude_vec.begin(),
                   [=](cv::Vec2f m) { return std::sqrt(m.dot(m)); });
    float mid = median(amplitude_vec, status);

    std::vector<float>::iterator it_amplitude = amplitude_vec.begin();
    std::replace_if(status.begin(), status.end(),
                    [&](uchar i) {
                        bool is_outlier = ((*it_amplitude > ratio * mid) || mid > ratio * (*it_amplitude));
                        it_amplitude++;
                        return is_outlier;
                    },
                    0);
}
void FeaturePointsManager::mark_status_with_angle(const std::vector<cv::Vec2f>& motion, std::vector<uchar>& status,
                                                  float shift) {
    std::vector<float> angle_vec(motion.size());
    std::transform(motion.begin(), motion.end(), angle_vec.begin(),
                   [=](cv::Vec2f m) { return atan2(m[1], m[0]) * 180 * M_1_PI; });

    float mid = median(angle_vec, status);

    std::vector<float>::iterator it_angle = angle_vec.begin();
    std::replace_if(status.begin(), status.end(),
                    [&](uchar i) {
                        bool is_outlier = ((*it_angle < mid - shift) || (*it_angle > mid + shift));
                        it_angle++;
                        return is_outlier;
                    },
                    0);
}
std::vector<cv::Vec2f> compute_pixel_motion(const std::vector<cv::Point2f>& old_feature_points,
                                            const std::vector<cv::Point2f>& new_feature_points) {
    std::vector<cv::Vec2f> result(old_feature_points.size());
    std::transform(old_feature_points.begin(), old_feature_points.end(), new_feature_points.begin(), result.begin(),
                   [=](cv::Point2f old_feature_point, cv::Point2f new_feature_point) {
                       return cv::Vec2f(new_feature_point.x - old_feature_point.x,
                                        new_feature_point.y - old_feature_point.y);
                   });
    return result;
}

float median(std::vector<float> data) {
    int n = data.size() / 2;
    std::nth_element(data.begin(), data.begin() + n, data.end());
    return data[n];
}

float median(std::vector<float> data, const std::vector<uchar>& status) {
    std::vector<float> good_data;

    for (int i = 0; i < status.size(); i++) {
        if (status[i]) {
            good_data.push_back(data[i]);
        }
    }

    return median(good_data);
}
