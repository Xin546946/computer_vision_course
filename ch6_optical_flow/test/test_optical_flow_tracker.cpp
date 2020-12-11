#include "bounding_box.h"
#include "opencv_utils.h"
#include "optical_flow_tracker.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv) {
    std::vector<cv::Mat> video;
    for (int id = 1; id < 914; id++) {
        cv::Mat img = cv::imread(argv[1] + std::to_string(id) + ".jpg", cv::IMREAD_GRAYSCALE);
        video.push_back(img);
    }

    // detect object in the firse image with template
    cv::Mat temp = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
    cv::Point2i center = template_matching(video[0], temp);

    BoundingBox bbox_init(center.x, center.y, template.cols, template.row);

    OpticalFlowTracker tracker(win_init);
    tracker.process(video);

    return 0;
}