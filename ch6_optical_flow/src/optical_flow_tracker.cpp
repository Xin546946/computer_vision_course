#include "optical_flow_tracker.h"
#include "bounding_box.h"
#include "opencv_utils.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>

void OpticalFlowTracker::process(BoundingBox initial_bbox, const std::vector<cv::Mat>& videos) {
    assert(!videos.empty());

    cv::Mat last_img = videos[0];
    // cv::equalizeHist(last_img, last_img);
    // cv::Mat mask = cv::Mat::zeros(videos[0].size(), videos[0].type());
    // cv::Rect mask_rect = cv::Rect(cv::Point(0, 0), mask.size());
    // cv::Rect bbox_rect = cv::Rect(initial_bbox.top_left(), cv::Size(initial_bbox.width(), initial_bbox.height()));
    // cv::Rect intersection = mask_rect & bbox_rect;
    // mask(intersection) = cv::Mat::ones(cv::Size(initial_bbox.width(), initial_bbox.height()), videos[0].type());

    feature_points_manager_.initialize(videos[0], initial_bbox);

    // cv::goodFeaturesToTrack(videos[0], prev_corners, 200, 0.01, 10, mask);

    // feature_points_manager_.set_tracking_results(prev_corners);

    for (int i = 1; i < videos.size(); i++) {
        //! 先跟踪, 再补点

        std::vector<cv::Point2f> prev_feature_points = feature_points_manager_.get_feature_points();

        std::vector<cv::Point2f> curr_feature_points;
        std::vector<uchar> status;
        std::vector<float> err;

        // aplly histogramm equalization in local
        BoundingBox bbox = feature_points_manager_.get_bbox();

        float w = 0.05;
        cv::Rect2i rect_local(bbox.top_left().x - 0.5 * w * bbox.width(), bbox.top_left().y - 0.5 * w * bbox.height(),
                              (1 + w) * bbox.width(), (1 + w) * bbox.height());

        cv::Mat local =
            get_sub_image_around(videos[i], rect_local.tl().x + 0.5 * rect_local.width,
                                 rect_local.tl().y + 0.5 * rect_local.height, rect_local.width, rect_local.height);

        cv::equalizeHist(local, local);
        local.copyTo(videos[i](rect_local));

        cv::calcOpticalFlowPyrLK(last_img, videos[i], prev_feature_points, curr_feature_points, status, err,
                                 cv::Size(11, 11), 3);

        feature_points_manager_.process_feature_points(videos[i], curr_feature_points, status);

        last_img = videos[i];
    }
}
