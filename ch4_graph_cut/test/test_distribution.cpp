#include "display.h"
#include "distribution.h"
#include "interaction_tool.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_COLOR);

    const auto& points = drag_to_get_fore_and_background_scribbles(img);

    Distribution distribution(img, points[0], points[1]);

    // get probability map
    cv::Mat fore_img = distribution.get_probability_map(0);
    cv::Mat back_img = distribution.get_probability_map(1);
    // visualize the probability map
    display_float_mat_img(fore_img, 0, "fore");
    display_float_mat_img(back_img, 0, "back");
    return 0;
}