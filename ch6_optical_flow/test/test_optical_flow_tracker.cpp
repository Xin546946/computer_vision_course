#include "bounding_box.h"
#include "opencv_utils.h"
#include "optical_flow_tracker.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    std::vector<cv::Mat> video;
    for (int id = 1; id < 914; id++) {
        cv::Mat img = cv::imread(argv[1] + std::to_string(id) + ".jpg", cv::IMREAD_GRAYSCALE);
        assert(!img.empty());
        video.push_back(img);
    }

    // detect object in the first image with template
    cv::Mat temp = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
    cv::Point2i center = template_matching(video[0], temp);

    // visualize bounding box in first frame
    cv::Mat vis_bbox = draw_bounding_box_vis_image(video[0], center.x, center.y, temp.cols, temp.rows);
    cv::imshow("img", vis_bbox);
    cv::waitKey(0);

    BoundingBox bbox_init(center.x, center.y, temp.cols, temp.rows);

    OpticalFlowTracker tracker;
    tracker.process(bbox_init, video);

    return 0;
}