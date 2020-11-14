#include "display.h"
#include "level_set_utils.h"
#include "sdf_map.h"
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

int main(int argc, char** argv) {
    // define and and initialize a sdf_map object
    cv::Mat image = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    disp_image(image, "face", 0);
    std::cout << image.size() << std::endl;
    image.convertTo(image, CV_64F);
    int size = 517;
    cv::Rect mask(180, 0, size, size);

    cv::Mat img = image(mask);
    std::cout << "w " << std::endl;
    assert(img.rows == img.cols && img.rows * img.cols % 2 == 1);

    double sigma = 100;
    cv::Mat gauss_kernel = gaussian_kernel(size, sigma);
    cv::Mat gauss_result = gauss_kernel.mul(img);
    cv::normalize(gauss_kernel, gauss_kernel, 0, 255, cv::NORM_MINMAX);
    gauss_kernel.convertTo(gauss_kernel, CV_8UC1);
    disp_image(gauss_kernel, "gaussian_kernel", 0);
    return 0;
}