#pragma once
#include <limits>

class GradientDescentBase {
   public:
    GradientDescentBase(float step_size);
    void run(int max_iteration);

   protected:
    virtual void initialize() = 0;
    virtual void update() = 0;
    virtual bool is_terminate(int current_iter, int max_iteration) const;

    virtual float compute_energy() = 0;
    virtual void roll_back_state() = 0;
    virtual void back_up_state() = 0;
    virtual void update_step_size(bool is_energy_decent);

    virtual void print_terminate_info() const;

    float step_size_ = 1e-10;
    float energy_ = std::numeric_limits<float>::max();
};