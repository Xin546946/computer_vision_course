#include "harris_corner.h"
#include "opencv_utils.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv) {
    cv::Mat img = read_img(argv[1], cv::IMREAD_GRAYSCALE);

    HarrisCornerDetector harris_corner_detector(img);
    harris_corner_detector.run();
    harris_corner_detector.show_result();

    return 0;
}