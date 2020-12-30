#pragma once
#include "histogram.h"
class MeanShiftTracking {
   public:
    MeanShiftTracking();
    void initialize();  // initialize mass center and histogram
    void update_mean_shift();
    double compute_energy();
    bool is_convergent();

   private:
    Histogram hist_;
};