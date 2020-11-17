#include "display.h"
#include "level_set_lbf.h"
#include "level_set_utils.h"
#include "sdf_map.h"
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

int main(int argc, char** argv) {
    // define and and initialize a sdf_map object
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::GaussianBlur(img, img, cv::Size(3, 3), 3);
    ParamLevelSetLBF param_level_set_lbf(
        5e-4, 5e-4, 0.01, 1e-2, 20, 1.2, 21,
        11);  // fore_weight, back_weight, eps,
              // step_size,l_w,g_w,w_size,sigma; 1.1,1.0,5e-2,15,1,2

    cv::Mat dx = do_sobel(img, 1);

    LevelSetLBF level_set_lbf(img, param_level_set_lbf);
    level_set_lbf.run(1e4);

    return 0;
}