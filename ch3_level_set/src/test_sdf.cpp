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
    cv::Mat sdf_with_contour = sdf_map.draw_contour(sdf_draw);
    disp_image(sdf_with_contour, "sdf", 0);

    return 0;
}