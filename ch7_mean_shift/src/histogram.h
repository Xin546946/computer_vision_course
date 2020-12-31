#pragma once
#include <cmath>
#include <vector>

class Histogram {
   public:
    /**
     * @brief Construct a new Histogram object, the input data should be in the range [min_value, max_value]
     * each bin cover the range [lhs,rhs)
     *
     * @param [in] num_bin
     * @param [in] min_value
     * @param [in] max_value
     */
    Histogram(int num_bin, double min_value, double max_value)
        : width_bin_((max_value - min_value + 1) / num_bin), hist_(num_bin) {
    }
    void add_data(double value, double weight = 1) {
        int bin = get_bin_id(value);
        hist_[bin] += weight;
    }

    int get_bin_id(const double value) {
        return std::floor(value / width_bin_);
    }

    double get_bin_height(const double id_bin) {
        return hist_[id_bin];
    }

    std::vector<double> get_hist() const {
        return hist_;
    }

   private:
    double width_bin_;  //! it should not be an integer
    std::vector<double> hist_;
};