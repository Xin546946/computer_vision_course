#include "gradient_descent_base.h"
#include <algorithm>

#include <iostream>

GradientDescentBase::GradientDescentBase(float step_size)
    : step_size_(step_size) {
}

void GradientDescentBase::run(int max_iteration) {
    initialize();
    last_energy_ = compute_energy();
    std::cout << "init energy : " << last_energy_ << "@@@@@@" << '\n';
    int current_iter = 0;
    while (!is_terminate(current_iter, max_iteration)) {
        current_iter++;
        back_up_state();
        update();

        float new_energy = compute_energy();
        std::cout.precision(5);
        std::cout << "current iteration: " << current_iter;
        std::cout << "current step size : " << step_size_;
        // if (new_energy < last_energy_) {
        if (true) {
            update_step_size(true);

            std::cout << "  engery decresed, accept update , "
                      << " new energy : " << new_energy
                      << " last energy : " << last_energy_;
            std::cout << " energy decresed for: " << new_energy - last_energy_
                      << '\n';
            last_energy_ = new_energy;

        } else {
            update_step_size(false);

            std::cout << "  engery incresed,   drop update , "
                      << " new energy : " << new_energy
                      << " last energy : " << last_energy_ << '\n';

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
    // step_size_ *= (is_energy_decent) ? 1.5 : 0.5;

    step_size_ = std::max(std::min(1e20f, step_size_), 1e-30f);
}