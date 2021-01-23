#include "haar_feature.h"
double compute_rect_integral_img_from_ul(cv::Mat integral_img, int row, int col, int width, int height) {
    // img(row,col) is 1
    return integral_img.at<double>(row, col) + integral_img.at<double>(row + height, col + width) -
           integral_img.at<double>(row, col + width) - integral_img.at<double>(row + height, col);
}

cv::Mat make_integral_img(const cv::Mat& img) {
    cv::Mat integral_img(cv::Mat::zeros(img.size(), CV_64FC1));
    // step 1: initialize first row and first col
    integral_img.at<double>(0, 0) = img.at<uchar>(0, 0);
    for (int r = 1; r < img.rows; r++) {
        integral_img.at<double>(r, 0) = img.at<uchar>(r, 0) + integral_img.at<double>(r - 1, 0);
    }
    for (int c = 1; c < img.cols; c++) {
        integral_img.at<double>(0, c) = img.at<uchar>(0, c) + integral_img.at<double>(0, c - 1);
    }
    assert(integral_img.size() == img.size());
    // step 2: use dynamic programming to determine the other elements
    for (int r = 1; r < img.rows; r++) {
        for (int c = 1; c < img.cols; c++) {
            integral_img.at<double>(r, c) = integral_img.at<double>(r - 1, c) + integral_img.at<double>(r, c - 1) -
                                            integral_img.at<double>(r - 1, c - 1) + img.at<uchar>(r, c);
        }
    }

    return integral_img;
}

cv::Mat compute_haar_feature_vector(DetectionWindow detection_window, cv::Mat integral_img, int x, int y) {
    cv::Mat feature_vec(detection_window.get_num_haar_rect(), 1, CV_64FC1);
    int id = 0;
    for (auto haar_rect : detection_window.get_haar_rects()) {
        double feature = 0.0;
        int row = y + haar_rect.ul().y;
        int col = x + haar_rect.ul().x;
        for (auto haar_sub_rect : haar_rect.get_haar_sub_rects()) {
            feature +=
                static_cast<double>(haar_sub_rect.sign_) *
                compute_rect_integral_img_from_ul(integral_img, row + haar_sub_rect.ul_.y, col + haar_sub_rect.ul_.x,
                                                  haar_sub_rect.width_, haar_sub_rect.height_);
        }
        feature_vec.at<double>(id, 0) = feature;
        id++;
    }
    return feature_vec;
}

Matrix<cv::Mat> compute_haar_feature_matrix(DetectionWindow detection_window, cv::Mat integral_img) {
    int rows = integral_img.rows - detection_window.get_height();
    int cols = integral_img.cols - detection_window.get_width();
    Matrix<cv::Mat> features_matrix(rows, cols);
    for (int r = 0; r < integral_img.rows - detection_window.get_height(); r++) {
        for (int c = 0; c < integral_img.cols - detection_window.get_width(); c++) {
            features_matrix.at(r, c) = compute_haar_feature_vector(detection_window, integral_img, c, r);
        }
    }
    return features_matrix;
}

HaarRect::HaarRect(int x, int y, int width, int height, cv::Mat indicator_matrix)
    : num_sub_rect_x_(indicator_matrix.cols),
      num_sub_rect_y_(indicator_matrix.rows),
      width_sub_rect_(width / indicator_matrix.cols),
      height_sub_rect_(height / indicator_matrix.rows),
      ul_(x, y) {
    for (int r = 0; r < indicator_matrix.rows; r++) {
        for (int c = 0; c < indicator_matrix.cols; c++) {
            haar_sub_rects_.emplace_back(indicator_matrix.at<char>(r, c),
                                         cv::Point2i(c * width_sub_rect_, r * height_sub_rect_), width_sub_rect_,
                                         height_sub_rect_);
        }
    }
}

void DetectionWindow::add_haar_rect(double x_ratio, double y_ratio, double width_ratio, double height_ratio,
                                    cv::Mat indicator_matrix) {
    int remainder_width = static_cast<int>(this->width_ * width_ratio) % indicator_matrix.cols;
    int remainder_height = static_cast<int>(this->height_ * height_ratio) % indicator_matrix.rows;
    int width = static_cast<int>(this->width_ * width_ratio) - remainder_width;
    int height = static_cast<int>(this->height_ * height_ratio) - remainder_height;
    haar_rects_.emplace_back(std::round(x_ratio * this->width_), std::round(y_ratio * this->height_), width, height,
                             indicator_matrix);
}

void DetectionWindow::show_all_sub_image(cv::Mat img, int x, int y) {
    for (HaarRect haar_rect : haar_rects_) {
        cv::Mat sub_img = get_sub_image_from_ul(img, x + haar_rect.ul().x, y + haar_rect.ul().y,
                                                haar_rect.get_num_sub_rect_x() * haar_rect.get_width_sub_rect(),
                                                haar_rect.get_num_sub_rect_y() * haar_rect.get_height_sub_rect());
        cv::imshow("sub img for feature extraction", sub_img);
        cv::waitKey(0);
    }
}