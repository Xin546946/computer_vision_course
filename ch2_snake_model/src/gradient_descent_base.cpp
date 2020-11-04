#include "gradient_descent_base.h"
#include <iostream>

void GradientDescentBase::run(int max_iteration) {
    initialize();
    int current_iter = 0;
    while (!is_terminate(current_iter, max_iteration)) {
        current_iter++;
        update();
    }
    print_terminate_info();
}

bool GradientDescentBase::is_terminate(int current_iter,
                                       int max_iteration) const {
    return (current_iter < max_iteration);
}
void GradientDescentBase::print_terminate_info() const {
    std::cout << "Iteration finished" << std::endl;
}