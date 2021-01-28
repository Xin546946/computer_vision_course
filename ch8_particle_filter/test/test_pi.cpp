#include "display.h"
#include "math_utils.h"
#include "opencv_utils.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    cv::Mat board(cv::Mat::ones(300, 300, CV_8UC1) * 255);
    cv::cvtColor(board, board, CV_GRAY2BGR);
    cv::circle(board, cv::Point(board.cols / 2, board.rows / 2), 150, cv::Scalar(0, 255, 0), 2);
    cv::imshow("pi result", board);

    float num_square = 1e10;
    float num_circle = 0;
    for (int i = 0; i < num_square; i++) {
        float x = generate_random_data(0.f, 1.f);
        float y = generate_random_data(0.f, 1.f);
        // std::cout << x << " " << y << '\n';
        float dist_square = std::pow(x - 0.5, 2) + std::pow(y - 0.5, 2);
        if (dist_square <= 0.25) {
            num_circle++;
            cv::drawMarker(board, cv::Point2f(x * 300.f, y * 300.f), cv::Scalar(0, 0, 255),
                           cv::MarkerTypes::MARKER_CROSS, 3);
            cv::imshow("pi result", board);
            cv::waitKey(1);
        } else {
            cv::drawMarker(board, cv::Point2f(x * 300.f, y * 300.f), cv::Scalar(255, 0, 0),
                           cv::MarkerTypes::MARKER_CROSS, 3);
            cv::imshow("pi result", board);
            cv::waitKey(1);
        }
    }
    float pi_estimation = num_circle / num_square * 4.0;
    std::cout << "estimated pi with " << num_square << "samples is" << pi_estimation << '\n';
    cv::waitKey(0);
    return 0;
}