/**
______________________________________________________________________
*********************************************************************
* @brief  This file is developed for the course of ShenLan XueYuan:
* Fundamental implementations of Computer Vision
* all rights preserved
* @author Xin Jin, Zhaoran Wu
* @contact: xinjin1109@gmail.com, zhaoran.wu1@gmail.com
*
______________________________________________________________________
*********************************************************************
**/
#pragma once
#include <limits>
#include <string>

class GradientDescentBase {
   public:
    GradientDescentBase(double step_size);
    void run(int max_iteration);

   protected:
    virtual void initialize() = 0;
    virtual void update() = 0;
    virtual bool is_terminate(int current_iter, int max_iteration) const;

    virtual double compute_energy() const = 0;
    virtual void roll_back_state() = 0;
    virtual void back_up_state() = 0;
    virtual void update_step_size(bool is_energy_decent);

    virtual void print_terminate_info() const;
    virtual std::string return_drive_class_name() const = 0;

    double step_size_ = 1e-10;
    double last_energy_ = std::numeric_limits<double>::max();
};