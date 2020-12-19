#include "mean_shift.h"
#include "data_base.h"

void MeanShift::run() {
    int max_it = 1e10;
    int it = 0;
    while (it++ < max_it && db_ptr_->is_convergent()) {
        db_ptr_->update_mass_center(win_ptr_);
        db_ptr_->visualize();
    }
}