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

// todo change the std::vector<cv::Point2d> to TestData

std::vector<cv::Point2d> generate_2dtest_data(int num_data, double a, double b, double std_dev, int num_outlier);
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

    auto datas = generate_2dtest_data(10, 1.0, 5.0, 1, 3);
    for (auto data : datas) {
        std::cout << data.y << " ";
    }
    std::cout << '\n';
    return 0;
}

struct TestData {
    std::vector<cv::Point2d> points_;
    cv::Point2d max() {
        double max_x = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::max();
        for (cv::Point2d point : points_) {
            if (point.y < max_y) {
                max_y = point.y;
            }
            if (point.x < max_x) {
                max_x = point.x;
            }
        }
        return cv::Point2d(max_x, max_y);
    }

    cv::Point2d min() {
        double min_x = std::numeric_limits<double>::min();
        double min_y = std::numeric_limits<double>::min();
        for (cv::Point2d point : points_) {
            if (point.y < min_y) {
                min_y = point.y;
            }
            if (point.x < min_x) {
                min_x = point.x;
            }
        }
        return cv::Point2d(min_x, min_y);
    }
};

std::vector<cv::Point2d> generate_2dtest_data(int num_data, double a, double b, double std_dev, int num_outlier) {
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::vector<cv::Point2d> points(num_data);
    for (int i = 0; i < num_data; i++) {
        double y = std::normal_distribution<double>(a * i + b, std_dev)(gen);
        points[i].x = i;
        points[i].y = y;
    }
    if (num_outlier) {
        std::vector<int> outlier_ids = generate_random_data(num_outlier, 0, num_data);
        for (int id : outlier_ids) {
            std::cout << " Outlier id: " << id << " ";
            points[id].y += std::normal_distribution<double>(a * id + b, 10.0 * std_dev)(gen);
        }
        std::cout << '\n';
    }
    return points;
}