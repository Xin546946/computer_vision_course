/**
______________________________________________________________________
*********************************************************************
* @brief This file is developed for the course of ShenLan XueYuan:
* Fundamental implementations of Computer Vision
* all rights preserved
* @author Xin Jin, Zhaoran Wu
* @contact: xinjin1109@gmail.com, zhaoran.wu1@gmail.com
*
______________________________________________________________________
*********************************************************************
**/
#include "bounding_box.h"
#include "opencv_utils.h"
#include "optical_flow_tracker.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    std::vector<cv::Mat> video;
    for (int id = 1; id < 875; id++) {
        cv::Mat img = cv::imread(argv[1] + std::to_string(id) + ".jpg", cv::IMREAD_GRAYSCALE);
        assert(!img.empty());
        video.push_back(img);
    }

    // detect object in the first image with template
    cv::Mat temp = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
    cv::Point2i center = template_matching(video[0], temp);

    float w = 0.0;
    BoundingBox bbox_init(center.x + w * temp.cols, center.y + w * temp.rows, (1 - 2 * w) * temp.cols,
                          (1 - 2 * w) * temp.rows);

    OpticalFlowTracker tracker(temp);
    tracker.process(bbox_init, video);

    return 0;
}