#include "display.h"
#include "level_set_utils.h"
#include "sdf_map.h"
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

int main(int argc, char** argv) {
    // define and and initialize a sdf_map object
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::GaussianBlur(img, img, cv::Size(3, 3), 3);
    int rows = img.rows;
    int cols = img.cols;
    cv::Point2d center(cols / 2.f, rows / 2.f);
    double radius = std::min(rows, cols) / 2.1f;
    SDFMap sdf_map(rows, cols, center, radius);
    ParamLevelSetCV param_level_set_cv(
        1, 1, 2.5, 7e-2, 0,
        1.2);  // fore_weight, back_weight, eps,
               // step_size,l_w,g_w; 1.1,1.0,5e-2,15,1,2

    cv::Mat dx = do_sobel(img, 1);
    cv::Mat vis_dx = get_float_mat_vis_img(dx);
    disp_image(vis_dx, "dx");

    LevelSetCV level_set_cv(img, param_level_set_cv);
    level_set_cv.run(1e4);

    return 0;
}