#include "gradient_descent_base.h"
#include "level_set_cv.h"

struct ParamLevelSetLBF : public ParamLevelSet {
    ParamLevelSetLBF(double forground_weight, double background_weight,
                     double eps, double step_size, double length_term_weight,
                     double gradient_term_weight, int window_size);
    int window_size_;
};

class LevelSetLBF : public GradientDescentBase {
   public:
    LevelSetLBF(cv::Mat image, const ParamLevelSetLBF& param);
};
