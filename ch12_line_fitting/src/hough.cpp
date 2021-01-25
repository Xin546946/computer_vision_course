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

// rho [pixel], theta[grad]
inline double compute_y(double x, double rho, double theta) {
    theta *= M_PI / 180.0;
    return (rho - x * std::cos(theta)) / std::sin(theta);
}

inline double grad_to_radian(double grad) {
    constexpr double radian_per_grad = M_PI / 180.0;
    return grad * radian_per_grad;
}

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
    double theta_reso = 1;
    double rho_reso = 1;
    cv::Mat accumulator = create_accumulator(theta_reso, rho_reso, max_rho);  // double
    voting(img_canny, accumulator);
    cv::Mat vis_accu = get_float_mat_vis_img(accumulator);
    cv::imshow("accumulator", vis_accu);
    cv::waitKey(0);
    std::vector<cv::Point> max_params = non_maxinum_suppress(static_cast<cv::Mat_<double>>(accumulator), 9, 15.0);
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

        int x1 = -1;
        int y1 = std::round(compute_y(x1, param.rho_, param.theta_));

        int x2 = img.cols;
        int y2 = std::round(compute_y(x2, param.rho_, param.theta_));

        cv::line(vis, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0));
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
    double max_rho = std::sqrt(img.rows * img.rows + img.cols * img.cols);

    double pixel_per_row = max_rho / accumulator.rows;
    double grad_per_col = 180.0 / accumulator.cols;

    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            if (img.at<uchar>(r, c)) {
                for (int col = 0; col < accumulator.cols; col++) {
                    double theta_in_grad = col * grad_per_col;
                    double theta_in_radians = grad_to_radian(theta_in_grad);

                    double rho = std::abs(c * std::cos(theta_in_radians) + r * std::sin(theta_in_radians));

                    int row = std::round(rho / pixel_per_row);
                    accumulator.at<double>(row, col)++;
                }
            }
        }
    }
    cv::GaussianBlur(accumulator, accumulator, cv::Size(3, 3), 3.0, 3.0);
}