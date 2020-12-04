#include "motion_seg.h"

void MotionSeg::process(const std::vector<cv::Mat>& videos) {
    assert(!videos.empty());

    const int cols = videos[0].cols;
    const int rows = videos[0].rows;
    gmm_map.resize(cols * rows);

    for (int i = 0; i < videos.size(); i++) {
        cv::Mat img_64;
        if (videos[0].type() != CV_64FC1) {
            videos[i].convertTo(img_64, CV_64FC1);
        }

        std::for_each()
    }
}