#include "display.h"
#include "gvf.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);

    // preprocess the input image
    cv::GaussianBlur(img, img, cv::Size(3, 3), 3, 3);

    cv::Mat grad_x_original, grad_y_original;
    cv::Sobel(img, grad_x_original, CV_64F, 1, 0, 3);
    cv::Sobel(img, grad_y_original, CV_64F, 0, 1, 3);

    double smooth_term = 1e8;
    double step_size = 1e-10;
    ParamGVF param_gvf(smooth_term, 21, step_size);  // TODO DELETE 21
    GVF gvf(grad_x_original, grad_y_original, param_gvf);

    int max_iteration_gvf = 1e4;
    gvf.run(max_iteration_gvf);  // parameter: max_iteration
    std::vector<cv::Mat> gvf_result = gvf.get_result_gvf();

    // display gvf result please set save as true if you want to save it
    bool save = true;
    display_gvf(gvf_result[0], gvf_result[1], 0, save);
}