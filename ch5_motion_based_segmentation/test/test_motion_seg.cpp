#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    std::vector<cv::Mat> video;
    for (int id = 1; id < 220; id++) {
        cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
        video.push_back(img);
    }

    MotionSeg ms;
    ms.process(video);

    return 0;
}