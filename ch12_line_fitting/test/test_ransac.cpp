#include "math_utils.h"
#include "opencv_utils.h"
#include "visualizer.h"
#include <array>
#include <iostream>
#include <limits>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <random>

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

DataSet2D generate_data_set_2d(int num_data, double a, double b, double std_dev, int num_outlier);
int main(int argc, char** argv) {
    DataSet2D data_set = generate_data_set_2d(1000, 1.0, 5.0, 10.0, 30);
    Visualizer vis(data_set.min_x_, data_set.max_x_, data_set.min_y_, data_set.max_y_);
    for (int i = 0; i < data_set.points_.size(); i++) {
        vis.add_point(data_set.points_[i].x, data_set.points_[i].y, data_set.is_inlier_[i]);
    }
    vis.show(0);

    return 0;
}

DataSet2D generate_data_set_2d(int num_data, double a, double b, double std_dev, int num_outlier) {
    std::random_device rd{};
    std::mt19937 gen{rd()};

    std::vector<cv::Point2d> points(num_data);

    DataSet2D data_set(points);

    data_set.min_x_ = 0.0;
    data_set.max_x_ = static_cast<double>(num_data) - 1.0;

    data_set.max_y_ = std::numeric_limits<double>::min();
    data_set.min_y_ = std::numeric_limits<double>::max();

    for (int i = 0; i < num_data; i++) {
        double y = std::normal_distribution<double>(a * i + b, std_dev)(gen);
        if (y > data_set.max_y_) {
            data_set.max_y_ = y;
        }
        if (y < data_set.min_y_) {
            data_set.min_y_ = y;
        }
        data_set.points_[i].x = i;
        data_set.points_[i].y = y;
    }
    if (num_outlier) {
        std::vector<int> outlier_ids = generate_random_data(num_outlier, 0, num_data - 1);
        std::cout << " Outlier id: ";
        for (int id : outlier_ids) {
            std::cout << id << " ";
            data_set.is_inlier_[id] = false;
            data_set.points_[id].y += std::normal_distribution<double>(a * id + b, 10.0 * std_dev)(gen);

            if (data_set.points_[id].y > data_set.max_y_) {
                data_set.max_y_ = data_set.points_[id].y;
            }

            if (data_set.points_[id].y < data_set.min_y_) {
                data_set.min_y_ = data_set.points_[id].y;
            }
        }
    }
    std::cout << '\n';
    std::cout << "Min x: " << data_set.min_x_ << '\n';
    std::cout << "Max x: " << data_set.max_x_ << '\n';
    std::cout << "Min y: " << data_set.min_y_ << '\n';
    std::cout << "Max y: " << data_set.max_y_ << '\n';

    for (int i = 0; i < data_set.points_.size(); i++) {
        std::cout.precision(6);
        std::cout << data_set.points_[i].x << " " << data_set.points_[i].y << ". is_inliner: " << data_set.is_inlier_[i]
                  << '\n';
    }
    return data_set;
}