#include "display.h"
#include "gvf.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], 0);
    std::cout << "Image channels: " << img.channels() << std::endl;
    cv::GaussianBlur(img, img, cv::Size(3, 3), 1, 1);
    cv::Mat grad_x_original, grad_y_original;
    cv::Sobel(img, grad_x_original, CV_32F, 1, 0, 3);
    cv::Sobel(img, grad_y_original, CV_32F, 0, 1, 3);
    ParamGVF param_gvf;
    GVF gvf(grad_x_original, grad_y_original, param_gvf);
    gvf.run(100);
    std::vector<cv::Mat> gvf_result = gvf.get_result_gvf();
    cv::Mat gvf_show = img.clone();
    cv::cvtColor(gvf_show, gvf_show, CV_GRAY2RGB);

    cv::Scalar color(0, 255, 0);
    draw_optical_flow(gvf_result[0], gvf_result[1], gvf_show, 8, 8, color);
    disp_image(gvf_show, "gvf");
    return 0;
}