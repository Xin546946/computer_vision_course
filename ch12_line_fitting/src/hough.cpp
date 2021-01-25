/**
______________________________________________________________________
*********************************************************************
* @brief This file is developed for the course of ShenLan XueYuan:
* Fundamental implementations of Computer Vision
* all rights preserved
* @author Xin Jin, Zhaoran Wu
* @contact: xinjin1109@gmail.com, zhaoran.wu1@gmail.com
*
______________________________________________________________________
*********************************************************************
**/
#include "hough.h"
#include "opencv_utils.h"

cv::Mat create_accumulator(double theta_reso, double rho_reso, double max_rho);

void voting(cv::Mat img, cv::Mat accumulator);

void draw_line(cv::Mat img, const std::vector<LineParam>& line_param);

std::vector<LineParam> line_detection(cv::Mat img) {
    assert(img.type() == CV_8UC1);
    std::cout << "img.rows: " << img.rows << " "
              << "img.cols: " << img.cols << '\n';
    cv::Mat img_canny;
    cv::Canny(img, img_canny, 50, 200, 3);
    assert(img_canny.type() == CV_8UC1);
    cv::imshow("canny img", img_canny);
    cv::waitKey(0);

    double max_rho = std::sqrt(img.rows * img.rows + img.cols * img.cols);
    double theta_reso = 1.0;
    double rho_reso = 4.0;
    cv::Mat accumulator = create_accumulator(theta_reso, rho_reso, max_rho);  // double
    voting(img_canny, accumulator);
    cv::Mat vis_accu = get_float_mat_vis_img(accumulator);
    cv::imshow("accumulator", vis_accu);
    cv::waitKey(0);
    std::vector<cv::Point> max_params = non_maxinum_suppress(static_cast<cv::Mat_<double>>(accumulator), 3, 20.0);
    std::vector<LineParam> line_param;
    for (cv::Point param : max_params) {
        line_param.emplace_back(param.y, param.x);
    }

    return line_param;
}

void draw_line(cv::Mat img, const std::vector<LineParam>& line_param) {
    cv::Mat vis = img.clone();
    cv::cvtColor(vis, vis, CV_GRAY2BGR);
    std::cout << "Num of line_param: " << line_param.size() << '\n';
    for (LineParam param : line_param) {
        std::cout << param.rho_ << " " << param.theta_ << '\n';
        cv::line(vis,
                 cv::Point(-1, param.rho_ + std::cos(param.theta_ * M_PI / 180) / std::sin(param.theta_ * M_PI / 180)),
                 cv::Point(img.cols + 1,
                           param.rho_ + std::cos(param.theta_ * M_PI / 180) / std::sin(param.theta_ * M_PI / 180)),
                 cv::Scalar(0, 255, 0));
    }
    cv::imshow("Line detection", vis);
    cv::waitKey(0);
}

cv::Mat create_accumulator(double theta_reso, double rho_reso, double max_rho) {
    int rows = std::ceil(max_rho / rho_reso);
    int cols = std::ceil(180.0 / theta_reso);
    std::cout << "rows: " << rows << " "
              << "cols: " << cols << '\n';
    return cv::Mat::zeros(rows, cols, CV_64FC1);
}

void voting(cv::Mat img, cv::Mat accumulator) {
    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            if (img.at<uchar>(r, c)) {
                for (int theta = 0; theta < accumulator.cols; theta++) {
                    accumulator.at<double>(std::round(180.0 / accumulator.cols * theta),
                                           std::round(c * std::cos(theta / accumulator.cols * M_PI) +
                                                      r * std::sin(theta / accumulator.cols * M_PI)))++;
                }
            }
        }
    }
    cv::GaussianBlur(accumulator, accumulator, cv::Size(3, 3), 3.0, 3.0);
}