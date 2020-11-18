#include "display.h"
#include "height_map.h"
#include "level_set_multi_phase.h"
#include "level_set_utils.h"
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

int main(int argc, char** argv) {
    // define and and initialize a height_map object
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::GaussianBlur(img, img, cv::Size(3, 3), 3);
    ParamLevelSetMP param_level_set_mp(
        5e-2, 5e-2, 5e-2, 5e-2, 1.0, 2e-2, 20,
        1.2);  // fore_weightssss, back_weight, eps,
               // step_size,l_w,g_w; 1.1,1.0,5e-2,15,1,2

    LevelSetMP level_set_mp(img, param_level_set_mp);
    level_set_mp.run(1e4);

    return 0;
}
