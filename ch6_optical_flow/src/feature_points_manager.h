#pragma once
#include "bounding_box.h"
#include <opencv2/highgui/highgui.hpp>
#include <vector>
class FeaturePointsManager {
   public:
    FeaturePointsManager() = default;

    void initialize(cv::Mat img, BoundingBox initial_bbox);
    void extract_new_feature_points(cv::Mat img);
    void process_feature_points(cv::Mat img, const std::vector<cv::Point2f>& feature_points,
                                std::vector<uchar>& status);

    std::vector<cv::Point2f> get_feature_points() const {
        return feature_points_;
    };

    BoundingBox get_bbox() const {
        return bbox_;
    };

   private:
    // void set_bounding_box(BoundingBox bbox);
    // std::vector<cv::Point2f> extract_feature_points(cv::Mat img, cv::Mat mask);
    // cv::Mat compute_curr_mask(cv::Mat img, BoundingBox initial_bbox);
    cv::Mat compute_mask(int rows, int cols);
    void update_status(const std::vector<cv::Vec2f>& motion, std::vector<uchar>& status);
    void update_bbox(const std::vector<cv::Vec2f>& motion, std::vector<uchar>& status);
    void update_feature_points(const std::vector<cv::Point2f>& new_feature_points, std::vector<uchar>& status);
    // void adjust_bbox();  // todo need to be discussed

    void mark_status_with_amplitude(const std::vector<cv::Vec2f>& motion, std::vector<uchar>& status, float rate);
    void mark_status_with_angle(const std::vector<cv::Vec2f>& motion, std::vector<uchar>& status, float rate);
    // void delete_with_status(std::vector<cv::Point2f> new_feature_points, const std::vector<uchar>& status);
    // void set_feature_points(std::vector<cv::Point2f> new_feature_points);

    void visualize(cv::Mat img, const std::vector<cv::Point2f>& feature_points_at_new_position);
    bool is_enough_points() const {
        return feature_points_.size() > 8;
    };

    std::vector<cv::Point2f> feature_points_;
    BoundingBox bbox_;
};

std::vector<cv::Point2f>& operator+=(const std::vector<cv::Point2f>& feature_points_1,
                                     const std::vector<cv::Point2f>& feature_points_2);