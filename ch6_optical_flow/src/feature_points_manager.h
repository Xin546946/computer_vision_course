#pragma once
#include "bounding_box.h"
#include <opencv2/highgui/highgui.hpp>
#include <vector>
class FeaturePointsManager {
   public:
    FeaturePointsManager();

    void initialize(cv::Mat img, BoundingBox initial_bbox);

    void extract_new_feature_points(cv::Mat img);

    void set_tracking_results(const std::vector<cv::Point2f>& corners, const std::vector<uchar>& status);
    std::vector<cv::Point2f> get_feature_points() const {
        return feature_points_;
    };
    BoundingBox get_bbox() const {
        return bbox_;
    };

   private:
    void set_bounding_box(BoundingBox bbox);
    std::vector<cv::Point2f> get_feature_points() const {
        return feature_points_;
    };
    // std::vector<cv::Point2f> extract_feature_points(cv::Mat img, cv::Mat mask);
    cv::Mat compute_curr_mask(cv::Mat img, BoundingBox initial_bbox);
    cv::Mat compute_mask(int rows, int cols);

    void update_bbox();
    void delete_bad_feature_points(const std::vector<cv::Point2f>& tracked_feature_points,
                                   const std::vector<uchar>& status);
    // void delete_with_rigid_body();
    std::vector<float>& compute_with_amplitude(const std::vector<cv::Point2f>& tracked_feature_points);
    std::vector<float>& compute_with_direction(const std::vector<cv::Point2f>& tracked_feature_points);
    void delete_with_direction(const std::vector<float>& angle);
    void delete_with_amplitude(const std::vector<float>& amplitude);
    void delete_with_status(const std::vector<uchar>& status);
    bool is_enough_points() {
        return feature_points_.size() > 8;
    };
    void process_num_feature_points();
    void process_bad_feature_points(const std::vector<cv::Point2f>& tracked_feature_points,
                                    const std::vector<uchar>& status);

    std::vector<cv::Point2f> feature_points_{};
    BoundingBox bbox_;
    // cv::Mat img_;
};

std::vector<cv::Point2f>& operator+=(const std::vector<cv::Point2f>& feature_points_1,
                                     const std::vector<cv::Point2f>& feature_points_2)