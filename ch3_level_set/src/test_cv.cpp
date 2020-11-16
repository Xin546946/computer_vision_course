#include "display.h"
#include "level_set_utils.h"
#include "sdf_map.h"
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

int main(int argc, char** argv) {
    // define and and initialize a sdf_map object
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::GaussianBlur(img, img, cv::Size(3, 3), 3);
    ParamLevelSetCV param_level_set_cv(
        1e-2, 1e-2, 1.0, 1e-2, 20,
        1.2);  // fore_weight, back_weight, eps,
               // step_size,l_w,g_w; 1.1,1.0,5e-2,15,1,2

    cv::Mat dx = do_sobel(img, 1);

    LevelSetCV level_set_cv(img, param_level_set_cv);
    level_set_cv.run(1e4);

    return 0;
}