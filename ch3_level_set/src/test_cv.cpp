#include "display.h"
#include "level_set_utils.h"
#include "sdf_map.h"
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

int main(int argc, char** argv) {
    // define and and initialize a sdf_map object
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);

    int rows = img.rows;
    int cols = img.cols;
    cv::Point2d center(cols / 2.f, rows / 2.f);
    double radius = std::min(rows, cols) / 4.f;
    SDFMap sdf_map(rows, cols, center, radius);
    ParamLevelSetCV param_level_set_cv(
        1, 1, 1.5, 1e-2, 0,
        0);  // fore_weight, back_weight, eps, step_size,l_w,g_w;

    cv::Mat dx = do_sobel(img, 1);
    cv::Mat vis_dx = get_float_mat_vis_img(dx);
    disp_image(vis_dx, "dx");

    LevelSetCV level_set_cv(img, param_level_set_cv);
    level_set_cv.run(1e4);

    return 0;
}