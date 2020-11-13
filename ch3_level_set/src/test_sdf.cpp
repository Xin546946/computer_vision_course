#include "display.cpp"
#include <opencv2/core.hpp>
int main(char argc, char** argv) {
    // define and and initialize a sdf_map object
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    int rows = img.rows;
    int cols = img.cols;
    cv::Point2d center(rows / 2.f, cols / 2.f);
    double radius = std::min(rows, cols) / 4.f;
    SDFMap sdf_map(rows, cols, center, radius);
    cv::Mat apply_jetmap(sdf_map);
    return 0;
}