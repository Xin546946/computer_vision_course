#include "display.h"
#include "level_set_utils.h"
#include "sdf_map.h"
#include <opencv2/core.hpp>

int main(int argc, char** argv) {
    // define and and initialize a sdf_map object
    int size = 101;
    double sigma = 15;
    cv::Mat gauss_kernel = gaussian_kernel(size, sigma);
    disp_image(gauss_kernel, "gaussian_kernel", 0);
    return 0;
}