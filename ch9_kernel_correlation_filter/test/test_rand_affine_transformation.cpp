#include "math_utils.h"
#include "opencv_utils.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

cv::Mat cmpute_rand_affine_transformation(cv::Mat img) {
    float angle = generate_random_data(float(-M_PI), float(M_PI));
    std::cout << angle << '\n';
    double cos_angle = cos(angle);
    double sin_angle = sin(angle);
    cv::Mat_<double> rotation_matrix(2, 3);
    rotation_matrix << cos_angle, -sin_angle, 0, sin_angle, cos_angle, 0;

    cv::Mat_<double> translation_vec(2, 1);
    translation_vec << img.cols / 2, img.rows / 2;
    rotation_matrix.col(2) = translation_vec - (rotation_matrix.colRange(0, 2)) * translation_vec;
    std::cout << "Test function of colRange(0,1)" << rotation_matrix.colRange(0, 2) << '\n';
    std::cout << rotation_matrix << '\n';
    cv::Mat wraped;
    cv::warpAffine(img, wraped, rotation_matrix, img.size(), cv::BORDER_REFLECT);
    return wraped;
}

int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    float angle = generate_random_data(float(-M_PI), float(M_PI));
    std::cout << angle << '\n';
    double cos_angle = cos(angle);
    double sin_angle = sin(angle);
    cv::Mat_<double> rotation_matrix(2, 3);
    rotation_matrix << cos_angle, -sin_angle, 0, sin_angle, cos_angle, 0;

    cv::Mat_<double> translation_vec(2, 1);
    translation_vec << img.cols / 2, img.rows / 2;
    rotation_matrix.col(2) = translation_vec - (rotation_matrix.colRange(0, 2)) * translation_vec;
    std::cout << "Test function of colRange(0,1)" << rotation_matrix.colRange(0, 2) << '\n';
    std::cout << rotation_matrix << '\n';
    cv::Mat wraped;
    cv::warpAffine(img, wraped, rotation_matrix, img.size(), cv::BORDER_REFLECT);
    cv::imshow("warp_img", wraped);
    cv::waitKey(0);
    return 0;
}