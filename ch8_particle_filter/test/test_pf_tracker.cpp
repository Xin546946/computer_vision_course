#include "bounding_box.h"
#include "opencv_utils.h"
#include "pf_tracker.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    std::vector<cv::Mat> video;
    for (int id = 1; id < 251; id++) {
        cv::Mat img = read_img(argv[1] + std::to_string(id) + ".jpg", cv::IMREAD_GRAYSCALE);
        video.push_back(img);
    }
    cv::Mat temp = read_img(argv[2], cv::IMREAD_GRAYSCALE);

    cv::Point2i init_upper_left = template_matching(video.front(), temp);
    cv::Point2f init_center = cv::Point2f(init_upper_left.x + temp.cols / 2.0f, init_upper_left.y + temp.rows / 2.0f);

    BoundingBox init_bbox(init_upper_left.x, init_upper_left.y, temp.cols, temp.rows);

    PFTracker pf_tracker(temp, init_bbox, 300);
    pf_tracker.process(video);

    return 0;
}