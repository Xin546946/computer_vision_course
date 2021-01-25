#include "haar_feature.h"
#include <utility>
double compute_rect_integral_img_from_ul(cv::Mat integral_img, int row, int col, int width, int height) {
    // todo compute integral img from ul point(row, col) with an HarrRect(width,height)
    // hints: Robust Real-Time Face Detection Figure 3.
}

cv::Mat make_integral_img(const cv::Mat& img) {
    // todo make an integral image using dynamic programming
    // hints: Robust Real-Time Face Detection part 2.1
}

cv::Mat compute_haar_feature_vector(DetectionWindow detection_window, cv::Mat integral_img, int x, int y) {
    // todo compute all haar feature in an detection window of an integral img at position (x,y)
}

Matrix<cv::Mat> compute_haar_feature_matrix(DetectionWindow detection_window, cv::Mat integral_img) {
    // todo compute haar feature for each position of the inregral img
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