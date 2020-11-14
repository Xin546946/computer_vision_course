#include "display.h"
#include "level_set_utils.h"
#include "sdf_map.h"
#include <opencv2/core.hpp>

int main(int argc, char** argv) {
    // define and and initialize a sdf_map object
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    if (img.rows != img.cols && img.rows * img.cols % 2 != 1)
        int size = static_cast<int>(sqrt(img.rows * img.cols));
    cv::Rect const mask(0, size, 0, size);
    assert(img.rows == img.cols);
    int size = img.rows;
    double sigma = 1;
    cv::Mat gauss_kernel = gaussian_kernel(size, sigma);
    cv::Mat gauss_result = gauss_kernel.mul(img);

    disp_image(gauss_result, "gaussian_kernel", 0);
    return 0;
}