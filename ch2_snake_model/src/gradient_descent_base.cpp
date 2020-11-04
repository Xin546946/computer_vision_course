#include "gradient_descent_base.h"

void GradientDescentBase::run(int max_iteration) {
    initialize();
    int current_iter = 0;
    while (!is_terminate(current_iter, max_iteration)) {
        current_iter++;
        update();
    }
}

bool GradientDescentBase::is_terminate(int current_iter, int max_iteration) {
    return (current_iter < max_iteration);
}