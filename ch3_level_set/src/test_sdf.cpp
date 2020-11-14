#include "display.h"
#include "level_set_helper_function.h"
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
    cv::Mat sdf_with_contour = draw_points(
        sdf_draw, sdf_map.get_contour_points(), cv::Scalar(255, 255, 255));
    disp_image(sdf_with_contour, "sdf", 0);

    cv::Mat fore_back_ground = sdf_map.get_fore_background_label_map();
    disp_image(fore_back_ground, "fore- and background", 0);

    cv::Mat h_phi = heaviside(sdf_map, 2.5);
    disp_image(h_phi, "h_phi", 0);

    cv::Mat one_minues_h_phi = complementary_heaviside(sdf_map, 2.5);
    disp_image(one_minues_h_phi, "one_minus_h_phi", 0);

    double energy_grad_mag_map = sdf_map.get_gradient_magnitude_level_set();
    std::cout << energy_grad_mag_map << std::endl;
    return 0;
}