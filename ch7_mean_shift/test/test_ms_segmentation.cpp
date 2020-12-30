#include "mean_shift_segmentation.h"
#include "opencv_utils.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    cv::Mat img = read_img(argv[1], cv::IMREAD_COLOR);

    cv::resize(img, img, cv::Size(100, 100));

    MeanShiftSeg mss(50);
    mss.process(img);

    return 0;
}