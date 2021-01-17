#include "harris_corner.h"
#include "opencv_utils.h"

cv::Matx22d compute_M(cv::Mat grad_x, cv::Mat grad_y) {
    cv::Mat gaussian_kernel = get_gaussian_kernel(7, 3.0);

    grad_x = grad_x.mul(gaussian_kernel);
    grad_y = grad_y.mul(gaussian_kernel);

    double element1 = grad_x.dot(grad_x);
    double element4 = grad_y.dot(grad_y);
    double element23 = grad_x.dot(grad_y);

    cv::Matx22d result(element1, element23, element23, element4);

    return result;
}

void HarrisCornerDetector::run() {
    cv::Mat img_grad_x, img_grad_y;
    assert(img_64f_.type() == CV_64FC1);
    cv::Sobel(img_64f_, img_grad_x, -1, 1, 0, 3);
    cv::Sobel(img_64f_, img_grad_y, -1, 0, 1, 3);

    double img_grad_x_mean = cv::mean(img_grad_x)[0];
    double img_grad_y_mean = cv::mean(img_grad_y)[0];
    assert(img_grad_x.type() == CV_64F);

    cv::Mat grad_x_zero_mean = img_grad_x - img_grad_x_mean * cv::Mat::ones(img_grad_x.size(), img_grad_x.type());
    cv::Mat grad_y_zero_mean = img_grad_y - img_grad_y_mean * cv::Mat::ones(img_grad_y.size(), img_grad_y.type());

    assert(grad_x_zero_mean.type() == CV_64FC1);

    for (int r = 3; r < img_64f_.rows + 3; r++) {
        for (int c = 3; c < img_64f_.cols + 3; c++) {
            cv::Mat grad_x_img_roi = get_sub_image_around(grad_x_zero_mean, c, r, 7, 7);
            cv::Mat grad_y_img_roi = get_sub_image_around(grad_y_zero_mean, c, r, 7, 7);

            cv::Matx22d M = compute_M(grad_x_img_roi, grad_y_img_roi);

            // cv::Matx22d e_value, e_vector;
            // cv::eigen(M, e_value, e_vector);

            double k = 0.05;
            double R = cv::determinant(M) - k * (cv::trace(M)) * (cv::trace(M));

            if (R > 20.0) {
                cv::circle(result_, cv::Point(r, c), 1, cv::Scalar(0, 0, 255), 1);
            }
        }
    }
}