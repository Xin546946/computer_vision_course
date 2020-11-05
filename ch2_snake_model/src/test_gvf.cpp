#include "display.h"
#include "gvf.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], 0);
    std::cout << "Image channels: " << img.channels() << std::endl;
    cv::GaussianBlur(img, img, cv::Size(7, 7), 7, 7);
    disp_image(img, "gauss", 0);
    cv::Mat grad_x_original, grad_y_original;

    cv::Sobel(img, grad_x_original, CV_32F, 1, 0, 3);
    cv::Sobel(img, grad_y_original, CV_32F, 0, 1, 3);

    ParamGVF param_gvf(0.2, 3, 0.1);  // mu , sigma, init step size
    GVF gvf(grad_x_original, grad_y_original, param_gvf);
    gvf.run(1000);
    return 0;
}