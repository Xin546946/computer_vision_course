#include "display.h"
#include "sdf_map.h"
#include <opencv2/core.hpp>

int main(int argc, char** argv) {
    // define and and initialize a sdf_map object
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    int rows = img.rows;
    int cols = img.cols;
    cv::Point2d center(cols / 2.f, rows / 2.f);
    double radius = std::min(rows, cols) / 4.f;
    SDFMap sdf_map(rows, cols, center, radius);

    cv::Mat sdf_draw = draw_sdf_map(sdf_map);
    disp_image(sdf_draw, "sdf", 0);
    cv::Mat fore_back_ground = sdf_map.get_fore_background_label_map();
    disp_image(fore_back_ground, "fore- and background", 0);
    double energy_grad_mag_map = sdf_map.get_gradient_magnitude_level_set();
    std::cout << energy_grad_mag_map << std::endl;
    return 0;
}