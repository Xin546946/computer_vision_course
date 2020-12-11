#include "optical_flow_tracker.h"
#include "bounding_box.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>

void OpticalFlowTracker::process(BoundingBox initial_bbox, const std::vector<cv::Mat>& videos) {
    assert(!videos.empty());

    std::vector<cv::Point2f> prev_corners;
    std::vector<cv::Point2f> curr_corners;
    std::vector<uchar> status;
    std::vector<float> err;

    cv::Mat last_img = videos[0];

    // cv::Mat mask = cv::Mat::zeros(videos[0].size(), videos[0].type());
    // cv::Rect mask_rect = cv::Rect(cv::Point(0, 0), mask.size());
    // cv::Rect bbox_rect = cv::Rect(initial_bbox.top_left(), cv::Size(initial_bbox.width(), initial_bbox.height()));
    // cv::Rect intersection = mask_rect & bbox_rect;
    // mask(intersection) = cv::Mat::ones(cv::Size(initial_bbox.width(), initial_bbox.height()), videos[0].type());

    prev_corners = feature_points_manager_.get_feature_points(last_img, initial_bbox);

    // cv::goodFeaturesToTrack(videos[0], prev_corners, 200, 0.01, 10, mask);

    // feature_points_manager_.set_tracking_results(prev_corners);

    for (int i = 1; i < videos.size(); i++) {
        cv::calcOpticalFlowPyrLK(last_img, videos[i], prev_corners, curr_corners, status, err, cv::Size(21, 21), 3);
        feature_points_manager_.set_tracking_results(curr_corners, status);
        // curr_corners = feature_points_manager_.delete_bad_feature_points(status);
        feature_points_manager_.update_box();
        feature_points_manager_.get_bbox();
        last_img = videos[i];
    }
}