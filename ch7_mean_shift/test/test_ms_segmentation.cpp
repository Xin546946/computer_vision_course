#include "ms_segmentation.h"
#include "opencv_utils.h"
#include "visualizer_3d.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>

int main(int argc, char** argv) {
    cv::Mat img = read_img(argv[1], cv::IMREAD_COLOR);

    cv::resize(img, img, cv::Size(100, 100));

    Vis3D* visualizer_ptr = new Vis3D();
    MeanShiftSeg mss(50, visualizer_ptr);  // mss take the ownership of visualizer_ptr;

    std::thread vis_thread(&Vis3D::visualize, std::ref(*visualizer_ptr));
    mss.process(img);

    vis_thread.join();

    return 0;
}