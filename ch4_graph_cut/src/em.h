#pragma once

class EMBase {
   public:
    EMBase() = default;
    void run(int max_iteration);

   protected:
    virtual void initialize() = 0;
    virtual void update_e_step() = 0;
    virtual void update_m_step() = 0;

    // virtual double compute_energy() = 0;

    virtual void print_terminate_info() const;
};