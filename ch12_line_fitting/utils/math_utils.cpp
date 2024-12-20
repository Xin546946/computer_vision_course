#include "math_utils.h"
#include <iostream>

DataSet2D generate_data_set_2d(int num_data, double a, double b, double std_dev, int num_outlier, bool print_info) {
    std::random_device rd{};
    std::mt19937 gen{rd()};

    std::vector<cv::Point2d> points(num_data);

    DataSet2D data_set(points);

    data_set.min_x_ = -data_set.num_ / 2;
    data_set.max_x_ = data_set.num_ + data_set.min_x_;

    data_set.max_y_ = std::numeric_limits<double>::min();
    data_set.min_y_ = std::numeric_limits<double>::max();
    for (int i = 0; i < data_set.num_; i++) {
        double y = std::normal_distribution<double>(a * (i + data_set.min_x_) + b, std_dev)(gen);
        if (y > data_set.max_y_) {
            data_set.max_y_ = y;
        }
        if (y < data_set.min_y_) {
            data_set.min_y_ = y;
        }
        data_set.points_[i].x = data_set.min_x_ + i;
        data_set.points_[i].y = y;
    }
    if (num_outlier) {
        std::vector<int> outlier_ids = generate_random_data(num_outlier, 0, num_data - 1);
        // std::cout << " Outlier id: ";
        for (int id : outlier_ids) {
            std::cout << id << " ";
            data_set.is_inlier_[id] = false;
            int sign = (2 * (id & 0x1) - 1);
            data_set.points_[id].y +=
                std::normal_distribution<double>(data_set.points_[id].y + sign * 3 * std_dev, 3 * std_dev)(gen);

            if (data_set.points_[id].y > data_set.max_y_) {
                data_set.max_y_ = data_set.points_[id].y;
            }

            if (data_set.points_[id].y < data_set.min_y_) {
                data_set.min_y_ = data_set.points_[id].y;
            }
        }
    }
    std::cout << '\n';
    if (print_info) {
        std::cout << "Min x: " << data_set.min_x_ << '\n';
        std::cout << "Max x: " << data_set.max_x_ << '\n';
        std::cout << "Min y: " << data_set.min_y_ << '\n';
        std::cout << "Max y: " << data_set.max_y_ << '\n';

        for (int i = 0; i < data_set.points_.size(); i++) {
            std::cout.precision(6);
            std::cout << data_set.points_[i].x << " " << data_set.points_[i].y
                      << ". is_inliner: " << data_set.is_inlier_[i] << '\n';
        }
    }
    return data_set;
}

std::vector<int> generate_random_data(int num, int min, int max) {
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::uniform_int_distribution<int> uniform_dist(min, max);
    std::vector<int> data(num);
    for (int i = 0; i < num; i++) {
        data[i] = uniform_dist(gen);
    }
    return data;
}

std::vector<float> generate_random_data(int num, float min, float max) {
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::uniform_real_distribution<float> uniform_dist(min, max);
    std::vector<float> data(num);
    for (int i = 0; i < num; i++) {
        data[i] = uniform_dist(gen);
    }
    return data;
}

float generate_random_data(float min, float max) {
    return generate_random_data(1, min, max)[0];
}

int generate_random_data(int min, int max) {
    return generate_random_data(1, min, max)[0];
}

std::vector<double> generate_gmm_data(int num_want, const std::vector<double>& means, const std::vector<double>& vars,
                                      const std::vector<double>& weights) {
    assert(means.size() == vars.size() && means.size() == weights.size());
    int sz = means.size();
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::vector<std::normal_distribution<double>> dists;
    for (int i = 0; i < sz; i++) {
        dists.emplace_back(means[i], vars[i]);
    }

    std::vector<double> result;

    for (int n = 0; n < num_want; n++) {
        double tmp = 0.0;
        for (int i = 0; i < sz; i++) {
            tmp += weights[i] * dists[i](gen);
        }
        result.push_back(tmp);
    }

    return result;
}

template <>
cv::Vec2f mean(const std::vector<cv::Vec2f>& vec) {
    assert(!vec.empty());
    cv::Vec2f sum = std::accumulate(vec.begin(), vec.end(), vec[0]) - vec[0];

    return cv::Vec2f(sum[0] / vec.size(), sum[1] / vec.size());
}