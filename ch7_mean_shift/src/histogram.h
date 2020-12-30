#pragma once
#include <cmath>
#include <vector>

class Histogram {
   public:
    Histogram(int num_bin, double min_value, double max_value)
        : num_bin_(num_bin),
          min_value_(min_value),
          max_value_(max_value + 1),  //! fix the upper boundary problem.
          width_bin_((max_value_ - min_value_) / num_bin_),
          hist_(num_bin) {
        // get_width_bin();
    }
    void add_data(double value, double weight = 1) {
        int bin = get_bin(value);
        hist_[bin] += weight;
    }

    int get_bin(const double value) {
        return std::floor(value / width_bin_);
    }

    double get_bin_height(const double id_bin) {
        return hist_[id_bin];
    }

    std::vector<double> get_hist() const {
        return hist_;
    }

   private:
    // void get_width_bin() {
    //     width_bin_ = (max_value_ - min_value_) / num_bin_;
    // }

    int num_bin_;
    const double min_value_;
    const double max_value_;
    double width_bin_;  //! it should not be an integer
    std::vector<double> hist_;
};