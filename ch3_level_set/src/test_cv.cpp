#include "display.h"
#include "level_set_utils.h"
#include "sdf_map.h"
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

int main(int argc, char** argv) {
    // define and and initialize a sdf_map object
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);

    cv::Mat test_sobel_y = do_sobel_y(img);
    test_sobel_y = get_float_mat_vis_img(test_sobel_y);
    disp_image(test_sobel_y, "sobel y", 0);
    int rows = img.rows;
    int cols = img.cols;
    cv::Point2d center(cols / 2.f, rows / 2.f);
    double radius = std::min(rows, cols) / 4.f;
    SDFMap sdf_map(rows, cols, center, radius);
    ParamLevelSetCV param_level_set_cv(
        1, 1.9, 1.0, 2e-1, 0.2,
        1e-5);  // fore_weight, back_weight, eps, step_size,l_w,g_w;

    LevelSetCV level_set_cv(img, param_level_set_cv);
    level_set_cv.run(1e4);

    return 0;
}