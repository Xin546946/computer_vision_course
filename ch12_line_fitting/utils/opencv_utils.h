#pragma once
// #include "bounding_box.h"
// #include "opencv2/imgproc.hpp"
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

cv::Mat read_img(std::string img_path, cv::ImreadModes read_mode);

inline bool is_in_img(cv::Mat img, int row, int col) {
    return row < img.rows && row >= 0 && col < img.cols && col >= 0;
}

inline int pos_to_id(int row, int col, int step) {
    return row * step + col;
}

inline cv::Point id_to_pos(int id, int step) {
    return {id / step, id % step};
}

std::vector<cv::Mat> record_webcam();
/**
 * @brief get location of a temlate given, image and a template
 *
 * @param img
 * @param temp
 * @return cv::Point2i
 */
cv::Point2i template_matching(cv::Mat img, cv::Mat temp);
/**
 * @brief get a sub image, safe at boundary without autofilling
 *
 * @param image
 * @param x : x of window center
 * @param y : y of window center
 * @param width : widht of the window
 * @param height : height of the window
 * @return cv::Mat : sub im
 */
cv::Mat get_sub_image_around(cv::Mat image, int x, int y, int width, int height);
cv::Mat get_sub_image_from_ul(cv::Mat image, int x, int y, int width, int height);

cv::Mat draw_bounding_box_vis_image(cv::Mat image, float x, float y, float width, float height);

cv::Rect get_intersection_around(cv::Mat image, int x, int y, int width, int height);

cv::Rect get_intersection_from_ul(cv::Rect rect_img, int x, int y, int width, int height);
cv::Rect get_intersection_from_ul(cv::Mat image, int x, int y, int width, int height);

cv::Mat get_float_mat_vis_img(cv::Mat);

cv::Mat get_gaussian_kernel(int size, double sigma);

// cv::Mat get_sub_image(cv::Mat img, BoundingBox bbox);

template <typename T1, typename T2>
void put_val_from_ul(T1 val, T2 input_mat, int x_ul, int y_ul, int width, int height);

template <typename T>
void put_val_around(T val, cv::Mat input_mat, int x_center, int y_center, int width, int height);

template <typename T>
void draw_points(cv::Mat img, const std::vector<cv::Point_<T>>& points, cv::Scalar bgr = cv::Scalar(0, 0, 255));

template <typename T>
void draw_lines(cv::Mat img, const std::vector<cv::Point_<T>>& src, const std::vector<cv::Point_<T>>& target,
                cv::Scalar bgr = cv::Scalar(0, 0, 255), int width = 2);

template <typename T>
void draw_arrowed_lines(cv::Mat img, const std::vector<cv::Point_<T>>& src, const std::vector<cv::Point_<T>>& target,
                        cv::Scalar bgr = cv::Scalar(0, 0, 255), int width = 2);
template <typename T>
cv::Mat_<T> do_sobel(cv::Mat_<T> input, int flag = 0);

/*--------------------------------------------------------
#####################implementation: template function #####################
---------------------------------------------------------*/
template <typename T1, typename T2>
void put_val_from_ul(T1 val, T2 input_mat, int x_ul, int y_ul, int width, int height) {
    cv::Rect intersection = get_intersection_from_ul(input_mat, x_ul, y_ul, width, height);
    input_mat(intersection) = val * cv::Mat::ones(intersection.size(), input_mat.type());
}

template <typename T>
void put_val_around(T val, cv::Mat input_mat, int x_center, int y_center, int width, int height) {
    assert(width % 2 == 1 && height % 2 == 1);
    put_val_from_ul(val, input_mat, x_center - width / 2, y_center - height / 2, width, height);
}

template <typename T>
void draw_points(cv::Mat img, const std::vector<cv::Point_<T>>& points, cv::Scalar bgr) {
    std::for_each(points.begin(), points.end(), [&](const cv::Point_<T> point) { cv::circle(img, point, 1, bgr, 2); });
}

template <typename T>
void draw_lines(cv::Mat img, const std::vector<cv::Point_<T>>& src, const std::vector<cv::Point_<T>>& target,
                cv::Scalar bgr, int width) {
    auto it_target = target.begin();
    std::for_each(src.begin(), src.end(),
                  [&](const cv::Point_<T> point) { cv::line(img, point, *it_target++, bgr, width); });
}

template <typename T>
void draw_arrowed_lines(cv::Mat img, const std::vector<cv::Point_<T>>& src, const std::vector<cv::Point_<T>>& target,
                        cv::Scalar bgr, int width) {
    auto it_target = target.begin();
    std::for_each(src.begin(), src.end(),
                  [&](const cv::Point_<T> point) { cv::arrowedLine(img, point, *it_target++, bgr, width); });
}

/**
 * @brief
 *
 * @param input
 * @param flag  = 0 x , = 1 y
 * @return cv::Mat
 */
template <typename T>
cv::Mat_<T> do_sobel(cv::Mat_<T> input, int flag) {
    cv::Mat_<T> im = input.clone();
    assert(input.channels() == 1);
    // if (im.type() != CV_64FC1) {
    //     im.convertTo(im, CV_64FC1);
    // }

    cv::Mat_<T> output(im.size(), im.type());
    for (int r = 0; r < im.rows; r++) {
        for (int c = 0; c < im.cols; c++) {
            int r_dhs = r + flag;
            int c_rhs = c + (1 - flag);
            c_rhs = std::min(std::max(0, c_rhs), im.cols - 1);
            r_dhs = std::min(std::max(0, r_dhs), im.rows - 1);
            int r_uhs = r - flag;
            int c_lhs = c - (1 - flag);
            c_lhs = std::min(std::max(0, c_lhs), im.cols - 1);
            r_uhs = std::min(std::max(0, r_uhs), im.rows - 1);
            output(r, c) = 0.5 * (im(r_dhs, c_rhs) - im(r_uhs, c_lhs));
        }
    }
    return output;
}

template <typename T>
cv::Point_<T> calc_mid_point(cv::Point_<T> p1, cv::Point_<T> p2) {
    return cv::Point_<T>(0.5 * (p1.x + p2.x), 0.5 * (p1.y + p2.y));
}

bool is_good_mat(cv::Mat mat, std::string mat_name);

template <typename T>
std::vector<cv::Point2i> non_maxinum_suppress(cv::Mat_<T> input, int win_size,
                                              T threshold = std::numeric_limits<T>::min()) {
    std::vector<cv::Point2i> maximum_pos;

    assert(win_size & 0x1);
    int half_size = win_size / 2;

    for (int row = 0; row < input.rows; row++) {
        for (int col = 0; col < input.cols; col++) {
            bool local_max = true;
            T center_value = input(row, col);
            if (center_value < threshold) {
                continue;
            }
            for (int r_win = -half_size; r_win < half_size; r_win++) {
                for (int c_win = -half_size; c_win < half_size; c_win++) {
                    if (r_win == 0 && c_win == 0) {
                        continue;
                    }

                    int r = std::max(std::min(row + r_win, input.rows - 1), 0);
                    int c = std::max(std::min(col + c_win, input.cols - 1), 0);

                    local_max = local_max && (center_value >= input(r, c));
                }
            }
            if (local_max) {
                maximum_pos.emplace_back(col, row);
                put_val_around(std::numeric_limits<T>::min(), static_cast<cv::Mat>(input), col, row, win_size,
                               win_size);
            }
        }
    }
    return maximum_pos;
}

void draw_dashed_line(cv::Mat img, cv::Point p1, cv::Point p2, cv::Scalar color, int line_width,
                      cv::LineTypes line_type);