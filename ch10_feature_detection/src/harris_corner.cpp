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
    // step 1: compute x and y derivatives of image
    cv::Mat img_grad_x, img_grad_y;
    assert(img_64f_.type() == CV_64FC1);
    cv::Sobel(img_64f_, img_grad_x, -1, 1, 0, 3);
    cv::Sobel(img_64f_, img_grad_y, -1, 0, 1, 3);

    // step 2: subtract the mean from each image gradient
    double img_grad_x_mean = cv::mean(img_grad_x)[0];
    double img_grad_y_mean = cv::mean(img_grad_y)[0];
    assert(img_grad_x.type() == CV_64F);

    cv::Mat grad_x_zero_mean = img_grad_x - img_grad_x_mean * cv::Mat::ones(img_grad_x.size(), img_grad_x.type());
    cv::Mat grad_y_zero_mean = img_grad_y - img_grad_y_mean * cv::Mat::ones(img_grad_y.size(), img_grad_y.type());
    // step 3: compute M (covariance) matrix for each pixel
    // step 4: compute eigenvalues and eigenvectors or other criterions such as in slides
    assert(grad_x_zero_mean.type() == CV_64FC1);
    cv::Mat responses(img_.size(), CV_64FC1);
    for (int r = 3; r < img_64f_.rows - 3; r++) {
        for (int c = 3; c < img_64f_.cols - 3; c++) {
            cv::Mat grad_x_img_roi = get_sub_image_around(grad_x_zero_mean, c, r, 7, 7);
            cv::Mat grad_y_img_roi = get_sub_image_around(grad_y_zero_mean, c, r, 7, 7);

            cv::Matx22d M = compute_M(grad_x_img_roi, grad_y_img_roi);

            double k = 0.05;
            responses.at<double>(r - 3, c - 3) = cv::determinant(M) - k * (cv::trace(M)) * (cv::trace(M));
        }
    }
    // step 5: use threshold on eigenvalues to detect corners
    int window_size = 31;
    double threshold = 300.0;
    int border = (window_size - 1) / 2;
    for (int r = border; r < responses.rows - border; r++) {
        for (int c = border; c < responses.cols - border; c++) {
            double response = responses.at<double>(r, c);
            if (response > threshold) {
                bool local_max = true;
                for (int r_win = -border; r_win < border; r_win++) {
                    for (int c_win = -border; c_win < border; c_win++) {
                        if (r_win == 0 && c_win == 0) {
                            continue;
                        }
                        local_max = local_max && (response >= responses.at<double>(r + r_win, c + c_win));
                    }
                }
                if (local_max) {
                    cv::circle(result_, cv::Point(c, r), 5, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
                }
            }
        }
    }
}