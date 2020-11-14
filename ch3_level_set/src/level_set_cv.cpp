#include "level_set_cv.h"
#include "level_set_utils.h"

LevelSetCV::LevelSetCV(cv::Mat image, const ParamLevelSetCV& param)
    : GradientDescentBase(param_.step_size_),
      level_set_(image.rows, image.cols,
                 cv::Point(image.cols / 2, image.rows / 2),
                 std::min(image.rows, image.cols) / 2.5f),
      param_(param),
      image_(image.clone()),
      center_foreground_(0.0),
      center_background_(255.0) {
}
