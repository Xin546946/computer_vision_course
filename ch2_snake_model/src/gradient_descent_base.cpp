#include "gradient_descent_base.h"

#include <iostream>

void GradientDescentBase::run(int max_iteration) {
    initialize();
    float last_energy = computer_energy();
    int current_iter = 0;
    while (!is_terminate(current_iter, max_iteration)) {
        current_iter++;
        back_up_state();
        update();

        energy_ = computer_energy();
        if (energy_ < last_energy) {
            update_step_size(true);
            last_energy = energy_;
        } else {
            update_step_size(false);
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
}