#include "mean_shift.h"
#include "data_base.h"
#include <iostream>

MeanShift::MeanShift(std::unique_ptr<DataBase>& db_ptr) : db_ptr_(std::move(db_ptr)) {
}

void MeanShift::run() {
    int max_it = 1000;
    int it = 0;
    while (it++ < max_it && !db_ptr_->is_convergent()) {
        db_ptr_->visualize();
        std::cout << "current iteration :" << it << '\n';
        db_ptr_->back_up_mass_center();
        db_ptr_->update_mass_center();
    }
}