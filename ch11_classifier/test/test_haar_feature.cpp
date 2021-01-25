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
#include "haar_feature.h"
#include "opencv_utils.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
/**
 * @brief Compute haar feature via template as ground truth
 *
 * @param detect_win
 * @param intergral_img_temps
 * @return cv::Mat
 */
cv::Mat compute_feature_ground_truth(const DetectionWindow& detect_win,
                                     const std::vector<cv::Mat>& intergral_img_temps);

int main(int argc, char** argv) {
    Matrix<std::vector<double>> test(109, 10);
    std::cout << "test pass" << '\n';

    cv::Mat img = read_img(argv[1], cv::IMREAD_GRAYSCALE);
    std::vector<cv::Mat> temps;
    for (int i = 0; i < 7; i++) {
        cv::Mat temp = read_img(std::string(argv[2]) + "template" + std::to_string(i) + ".png", cv::IMREAD_GRAYSCALE);
        temps.push_back(temp);
    }

    DetectionWindow detection_window(temps[0].cols, temps[0].rows);

    detection_window.add_haar_rect(0.200, 0.350, 0.618, 0.200, (cv::Mat_<int>(1, 3) << -1, 1, -1));
    // todo make your own  haar feature

    // convert the img to integral img
    cv::Mat integral_img = make_integral_img(img);
    std::vector<cv::Mat> integral_img_temps;
    for (cv::Mat temp : temps) {
        cv::Mat integral_img_temp = make_integral_img(temp);
        integral_img_temps.push_back(integral_img_temp);
    }

    // compute haar feature (ground truth) for each haar rect
    cv::Mat feature_ground_truth = compute_feature_ground_truth(detection_window, integral_img_temps);
    // compute haar feature on the img via haar rects
    Matrix<cv::Mat> feature_matrix = compute_haar_feature_matrix(detection_window, integral_img);

    cv::Mat minus_dist_map(feature_matrix.rows_, feature_matrix.cols_, CV_64FC1);

    const double thres = 5e8;

    for (int r = 0; r < feature_matrix.rows_; r++) {
        for (int c = 0; c < feature_matrix.cols_; c++) {
            double dist = cv::norm(feature_ground_truth, feature_matrix.at(r, c), cv::NORM_L2SQR);
            minus_dist_map.at<double>(r, c) = (dist < thres) ? -dist : std::numeric_limits<double>::min();
        }
    }

    std::vector<cv::Point2i> max_pos = non_maxinum_suppress(minus_dist_map, 17);

    cv::cvtColor(img, img, CV_GRAY2BGR);
    for (cv::Point2i pos : max_pos) {
        cv::rectangle(img, cv::Rect2i(pos, cv::Size(detection_window.get_width(), detection_window.get_height())),
                      cv::Scalar(0, 255, 0), 2);
    }

    cv::imshow("detection result", img);
    cv::waitKey(0);

    return 0;
}

cv::Mat compute_feature_ground_truth(const DetectionWindow& detect_win,
                                     const std::vector<cv::Mat>& intergral_img_temps) {
    cv::Mat result(detect_win.get_num_haar_rect(), 1, CV_64FC1, 0.0);

    for (auto img : intergral_img_temps) {
        result += compute_haar_feature_vector(detect_win, img, 0, 0);
    }

    result /= static_cast<double>(intergral_img_temps.size());

    return result;
}