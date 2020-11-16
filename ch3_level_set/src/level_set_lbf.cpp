#include "level_set_lbf.h"

ParamLevelSetLBF::ParamLevelSetLBF(double forground_weight,
                                   double background_weight, double eps,
                                   double step_size, double length_term_weight,
                                   double gradient_term_weight, int window_size)
    : ParamLevelSet(forground_weight, background_weight, eps, step_size,
                    length_term_weight, gradient_term_weight),
      window_size_(window_size) {
}
