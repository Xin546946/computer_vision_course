#include "gradient_descent_base.h"
#include <algorithm>

#include <iostream>

GradientDescentBase::GradientDescentBase(float step_size)
    : step_size_(step_size) {
}

void GradientDescentBase::run(int max_iteration) {
    initialize();
    float last_energy = compute_energy();
    int current_iter = 0;
    while (!is_terminate(current_iter, max_iteration)) {
        current_iter++;
        back_up_state();
        update();

        energy_ = compute_energy();
        std::cout.precision(5);
        std::cout << "current step size : " << step_size_;
        if (energy_ < last_energy) {
            update_step_size(true);
            std::cout << "  engery decresed, accept update , "
                      << " current energy : " << energy_
                      << " energy diff: " << energy_ - last_energy << '\n';

            last_energy = energy_;
        } else {
            update_step_size(false);
            std::cout << "  engery incresed,   drop update , "
                      << " current energy : " << energy_
                      << " energy diff: " << energy_ - last_energy << '\n';

            roll_back_state();
        }
    }
    print_terminate_info();
}

bool GradientDescentBase::is_terminate(int current_iter,
                                       int max_iteration) const {
    return (current_iter >= max_iteration);
}
void GradientDescentBase::print_terminate_info() const {
    std::cout << "Iteration finished" << std::endl;
}

void GradientDescentBase::update_step_size(bool is_energy_decent) {
    step_size_ *= (is_energy_decent) ? 1.5f : 0.5f;
    step_size_ = std::max(std::min(1e20f, step_size_), 1e-30f);
}