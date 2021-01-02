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
    /**
     * @brief add a data to the histogramm
     *
     * @param [in] value
     * @param [in] weight
     */
    void add_data(double value, double weight = 1) {
        int bin = get_bin_id(value);
        hist_[bin] += weight;
    }
    /**
     * @brief return how many bins in the histogramm
     *
     * @return int
     */
    int num_bin() const {
        return hist_.size();
    }

    /**
     * @brief return the correspond bin's id given a value
     *
     * @param [in] value
     * @return int
     */
    int get_bin_id(double value) const {
        return std::floor(value / width_bin_);
    }

    /**
     * @brief return the height of a bin
     *
     * @param [in] id_bin
     * @return double
     */
    double get_bin_height(double id_bin) const {
        return hist_[id_bin];
    }

    /**
     * @brief Get the hist object
     *
     * @return std::vector<double>
     */
    std::vector<double> get_hist() const {
        return hist_;
    }

   private:
    double width_bin_;  // the width of a bin
    std::vector<double> hist_;
};