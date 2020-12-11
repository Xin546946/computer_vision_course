#pragma once
#include "bounding_box.h"
#include <opencv2/highgui/highgui.hpp>
#include <vector>
class FeaturePointsManager {
   public:
    FeaturePointsManager();

    void initialize(cv::Mat img, BoundingBox initial_bbox);
    std::vector<cv::Point2f> get_feature_points() const {
        return feature_points_;
    };
    void set_tracking_results(const std::vector<cv::Point2f>& corners, const std::vector<uchar>& status);
    void get_bbox() const;

   private:
    std::vector<cv::Point2f> feature_points_;
    void extract_feature_points(cv::Mat img);
    BoundingBox bbox_;
    cv::Mat compute_curr_mask();
    void update_bbox();
    void delete_bad_feature_points(const std::vector<uchar>& status);
    void delete_with_rigid_body();
    void delete_with_direction();
    void delete_with_step_length();
    void delete_with_status();
    void check_if_enough();
};