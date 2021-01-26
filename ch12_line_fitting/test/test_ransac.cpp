#include "math_utils.h"
#include "opencv_utils.h"
#include <array>
#include <iostream>
#include <limits>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <random>

struct DataSet2D {
    DataSet2D(std::vector<cv::Point2d> points) : points_(points) {
    }

    std::vector<cv::Point2d> points_;

    double max_x_;
    double max_y_;
    double min_x_;
    double min_y_;
};
// todo change the std::vector<cv::Point2d> to TestData
DataSet2D generate_2dtest_data(int num_data, double a, double b, double std_dev, int num_outlier);
int main(int argc, char** argv) {
    // use opencv coordinate
    cv::Mat data_map(cv::Mat::zeros(300, 300, CV_64FC1));
    cv::Mat point_data = data_map.clone();
    cv::line(data_map, cv::Point(0, data_map.rows), cv::Point(data_map.cols, 0), cv::Scalar(255, 255, 255), 2);
    for (int r = 0; r < data_map.rows; r += 3) {
        for (int c = 0; c < data_map.cols; c += 3) {
            if (data_map.at<double>(r, c)) {
                std::array<double, 2> mean{r, c};
                std::array<double, 2> stddev{50.0, 3.0};
                std::vector<std::array<double, 2>> gauss_data = generate_gauss_data<double, 2>(1, mean, stddev);
                for (auto data : gauss_data) {
                    int row = std::min(std::max(0.0, data[1]), static_cast<double>(data_map.rows));
                    row = row + generate_random_data(3, 100);
                    int col = std::min(std::max(0.0, data[0]), static_cast<double>(data_map.cols));
                    point_data.at<double>(row, col) = 1.0;
                }
            }
        }
    }

    cv::imshow("data", point_data);
    cv::waitKey(0);

    return 0;
}

DataSet2D generate_2dtest_data(int num_data, double a, double b, double std_dev, int num_outlier) {
    std::random_device rd{};
    std::mt19937 gen{rd()};

    std::vector<cv::Point2d> points(num_data);

    DataSet2D data_set(points);

    data_set.min_x_ = 0.0;
    data_set.max_x_ = static_cast<double>(num_data);

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
        std::vector<int> outlier_ids = generate_random_data(num_outlier, 0, num_data);
        for (int id : outlier_ids) {
            std::cout << " Outlier id: " << id << " ";

            points[id].y += std::normal_distribution<double>(a * id + b, 10.0 * std_dev)(gen);

            if (points[id].y > data_set.max_y_) {
                data_set.max_y_ = points[id].y;
            }

            if (points[id].y < data_set.min_y_) {
                data_set.min_y_ = points[id].y;
            }
        }
    }
    return points;
}