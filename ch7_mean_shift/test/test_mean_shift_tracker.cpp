#include "bounding_box.h"
#include "mean_shift_tracker.h"
#include "opencv_utils.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    std::vector<cv::Mat> video;
    for (int id = 1; id < 251; id++) {
        cv::Mat img = read_img(argv[1] + std::to_string(id) + ".jpg", cv::IMREAD_GRAYSCALE);
        assert(!img.empty());
        video.push_back(img);
    }

    cv::Mat temp = read_img(argv[2], cv::IMREAD_GRAYSCALE);
    std::cout << temp.type() << '\n';
    MeanShiftTracker mstracker;
    mstracker.process(video, temp);

    return 0;
}