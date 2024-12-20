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

static constexpr double radian_per_grad = M_PI / 180.0;
// rho [pixel], theta[grad]
inline double compute_y(double x, double rho, double theta) {
    theta *= radian_per_grad;
    return (rho - x * std::cos(theta)) / (std::sin(theta) + 1e-3);
}

inline double compute_x(double y, double rho, double theta) {
    theta *= radian_per_grad;
    return (rho - y * std::sin(theta)) / (std::cos(theta) + 1e-3);
}

inline double grad_to_radian(double grad) {
    return grad * radian_per_grad;
}

cv::Mat create_accumulator(double theta_reso, double rho_reso, double max_rho);

void voting(cv::Mat img, cv::Mat accumulator, double max_rho);
void draw_line(cv::Mat img, const std::vector<LineParam>& line_param);

std::vector<LineParam> line_detection(cv::Mat img, double theta_resolution, double rho_resoulution) {
    assert(img.type() == CV_8UC1);
    // step 1: use canny for extracting edges
    cv::Mat img_canny;
    cv::Canny(img, img_canny, 50, 200, 3);
    assert(img_canny.type() == CV_8UC1);
    cv::imshow("canny img", img_canny);
    cv::waitKey(0);
    // step 2: set parameter resolution
    double max_rho = std::sqrt(img.rows * img.rows + img.cols * img.cols);

    // step 3: set accumulator w.r.t. parameter space
    cv::Mat accumulator = create_accumulator(theta_resolution, rho_resoulution, max_rho);  // double

    // step 4: voting in the accumulator
    voting(img_canny, accumulator, max_rho);
    cv::Mat vis_accu;
    cv::normalize(accumulator, vis_accu, 0, 255, cv::NORM_MINMAX);

    // step 5: find local maximum
    std::vector<cv::Point> max_pos = non_maxinum_suppress(static_cast<cv::Mat_<double>>(accumulator), 51, 101.0);
    vis_accu.convertTo(vis_accu, CV_8UC1);
    cv::cvtColor(vis_accu, vis_accu, cv::COLOR_GRAY2BGR);

    for (auto param : max_pos) {
        cv::drawMarker(vis_accu, param, cv::Scalar(0, 0, 255), cv::MARKER_CROSS);
    }

    // cv::resize(vis_accu, vis_accu, cv::Size(1000 * vis_accu.cols / vis_accu.rows, 1000));
    cv::imshow("accumulator", vis_accu);
    cv::waitKey(0);

    std::vector<LineParam> line_param;
    for (cv::Point pos : max_pos) {
        line_param.emplace_back(pos.y * rho_resoulution, pos.x * theta_resolution);
    }

    return line_param;
}

void draw_line(cv::Mat img, const std::vector<LineParam>& line_param) {
    cv::Mat vis = img.clone();
    cv::cvtColor(vis, vis, CV_GRAY2BGR);
    std::cout << "Num of line_param: " << line_param.size() << '\n';
    for (LineParam param : line_param) {
        std::cout << param.rho_ << " " << param.theta_ << '\n';

        int x1, x2, y1, y2;

        if ((param.theta_ < 45 && param.theta_ > 315) || (param.theta_ > 135 && param.theta_ < 225)) {
            x1 = -1;
            y1 = std::round(compute_y(x1, param.rho_, param.theta_));

            x2 = img.cols;
            y2 = std::round(compute_y(x2, param.rho_, param.theta_));
        } else {
            y1 = -1;
            x1 = std::round(compute_x(y1, param.rho_, param.theta_));

            y2 = img.rows;
            x2 = std::round(compute_x(y2, param.rho_, param.theta_));
        }
        std::cout << " x1 , y1 :" << x1 << " ," << y1 << '\n';
        std::cout << " x2 , y2 :" << x2 << " ," << y2 << '\n';
        draw_dashed_line(vis, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
    cv::imshow("Line detection", vis);
    cv::waitKey(0);
}

cv::Mat create_accumulator(double theta_reso, double rho_reso, double max_rho) {
    int rows = std::ceil(max_rho / rho_reso);
    int cols = std::ceil(359.0 / theta_reso);
    std::cout << "rows: " << rows << " "
              << "cols: " << cols << '\n';
    return cv::Mat::zeros(rows, cols, CV_64FC1);
}

void voting(cv::Mat img, cv::Mat accumulator, double max_rho) {
    double pixel_per_row = max_rho / accumulator.rows;
    double grad_per_col = 359.0 / accumulator.cols;

    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            if (img.at<uchar>(r, c)) {
                for (int col = 0; col < accumulator.cols; col++) {
                    double theta_in_grad = col * grad_per_col;
                    double theta_in_radians = grad_to_radian(theta_in_grad);

                    double rho = c * std::cos(theta_in_radians) + r * std::sin(theta_in_radians);
                    if (rho < 0) continue;

                    int row = std::round(rho / pixel_per_row);

                    accumulator.at<double>(row, col)++;
                }
            }
        }
    }
}