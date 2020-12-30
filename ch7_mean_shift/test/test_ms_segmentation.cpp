#include "mean_shift_segmentation.h"
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

    Vis3D* vis_ptr = new Vis3D();
    MeanShiftSeg mss(50, vis_ptr);  // mss take the ownership of vis_ptr;

    std::thread vis_th(&Vis3D::visualize, std::ref(*vis_ptr));
    mss.process(img);

    vis_th.join();

    return 0;
}