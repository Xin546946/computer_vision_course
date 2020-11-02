#include "snake_model.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

void Snake_Model::run() {
    initial_contour();
    calcu_external_force_image();
    calcu_gvf();
    calcu_ballon_force();
    calcu_internal_force_matrix();
    for (int iter = 0; iter < Snake_Model::snake_move_param_.max_iterations_;
         iter++) {
        update_snake();
    }
}