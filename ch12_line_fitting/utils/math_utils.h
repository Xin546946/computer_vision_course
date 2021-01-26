#pragma once

#include <array>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <random>
#include <vector>

struct DataSet2D {
    DataSet2D(std::vector<cv::Point2d> points)
        : points_(points), num_(points_.size()), is_inlier_(points_.size(), true) {
    }

    std::vector<cv::Point2d> points_;

    int num_;

    std::vector<bool> is_inlier_;

    double max_x_ = std::numeric_limits<double>::max();
    double max_y_ = std::numeric_limits<double>::max();
    double min_x_ = std::numeric_limits<double>::min();
    double min_y_ = std::numeric_limits<double>::min();
};

DataSet2D generate_data_set_2d(int num_data, double a, double b, double std_dev, int num_outlier,
                               bool print_info = true);

template <typename T>
T mean(const std::vector<T>& vec) {
    assert(!vec.empty());
    T sum = std::accumulate(vec.begin(), vec.end(), vec[0]) - vec[0];

    return sum / vec.size();
}

std::vector<int> generate_random_data(int num, int min, int max);

std::vector<float> generate_random_data(int num, float min, float max);

float generate_random_data(float min, float max);

int generate_random_data(int min, int max);

std::vector<double> generate_gmm_data(int num_want, const std::vector<double>& means, const std::vector<double>& vars,
                                      const std::vector<double>& weights);

template <typename T, int Dim>
std::vector<std::array<T, Dim>> generate_gauss_data(int num, const std::array<T, Dim>& means,
                                                    const std::array<T, Dim>& stddev) {
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::array<std::normal_distribution<T>, Dim> normal_dists;
    for (int i = 0; i < Dim; i++) {
        normal_dists[i] = std::normal_distribution<T>(means[i], stddev[i]);
    }

    std::vector<std::array<T, Dim>> result(num);
    for (int n = 0; n < num; n++) {
        std::array<T, Dim> data;
        for (int i = 0; i < Dim; i++) {
            data[i] = normal_dists[i](gen);
        }
        result[n] = (data);
    }

    return result;
}

template <int Dim>
std::vector<std::array<int, Dim>> generate_nd_data(int num, int min, int max) {
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::array<std::uniform_int_distribution<int>, Dim> uniform_dists;
    for (int i = 0; i < Dim; i++) {
        uniform_dists[i] = std::uniform_int_distribution<int>(min, max);
    }

    std::vector<std::array<int, Dim>> result(num);
    for (int n = 0; n < num; n++) {
        std::array<int, Dim> data;
        for (int i = 0; i < Dim; i++) {
            data[i] = uniform_dists[i](gen);
        }
        result[n] = (data);
    }

    return result;
}
