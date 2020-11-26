#include "display.h"
#include "graph_cut.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_COLOR);

    GraphCut gc(img);
    gc.run();
    cv::Mat fore_img = gc.get_segmenation(SegType::FOREGROUND);
    cv::Mat back_img = gc.get_segmenation(SegType::BACKGROUND);

    return 0;
}