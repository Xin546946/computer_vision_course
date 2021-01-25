#include "opencv_utils.h"
#include <iostream>
#include <limits>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
double compute_rect_integral_img_from_ul(cv::Mat integral_img, int row, int col, int width, int height) {
    // img(row,col) is 1
    return integral_img.at<double>(row, col) + integral_img.at<double>(row + height, col + width) -
           integral_img.at<double>(row, col + width) - integral_img.at<double>(row + height, col);
}

cv::Mat make_integral_img(const cv::Mat& img) {
    cv::Mat integral_img(cv::Mat::zeros(img.size(), img.type()));
    // step 1: initialize first row and first col
    integral_img.at<double>(0, 0) = img.at<double>(0, 0);
    for (int r = 1; r < img.rows; r++) {
        integral_img.at<double>(r, 0) = img.at<double>(r, 0) + integral_img.at<double>(r - 1, 0);
    }
    for (int c = 1; c < img.cols; c++) {
        integral_img.at<double>(0, c) = img.at<double>(0, c) + integral_img.at<double>(0, c - 1);
    }

    // step 2: use dynamic programming to determine the other elements
    for (int r = 1; r < img.rows; r++) {
        for (int c = 1; c < img.cols; c++) {
            integral_img.at<double>(r, c) = integral_img.at<double>(r - 1, c) + integral_img.at<double>(r, c - 1) -
                                            integral_img.at<double>(r - 1, c - 1) + img.at<double>(r, c);
        }
    }
    return integral_img;
}

int main(int argc, char** argv) {
    cv::Mat img = read_img(argv[1], cv::IMREAD_GRAYSCALE);
    cv::Mat temp = read_img(argv[2], cv::IMREAD_GRAYSCALE);

    // check the rows and cols of template are odd number and adjust template
    int width = temp.cols;
    int height = temp.rows;
    width = width % 2 ? width - 1 : width;
    height = height % 2 ? height - 1 : height;
    temp = get_sub_image_from_ul(temp, 0, 0, width, height);
    temp.convertTo(temp, CV_64FC1);
    // generate ground truth
    cv::Mat upper_half = cv::Mat::ones(cv::Size(width, height / 2), CV_64FC1);
    cv::Mat lower_half = -upper_half;
    cv::Mat kernel_hor;
    cv::vconcat(upper_half, lower_half, kernel_hor);

    cv::Mat left_half = cv::Mat::ones(cv::Size(width / 2, height), CV_64FC1);
    cv::Mat right_half = -left_half;
    cv::Mat kernel_ver;
    cv::hconcat(left_half, right_half, kernel_ver);

    cv::Mat upper_left = cv::Mat::ones(cv::Size(width / 2, height / 2), CV_64FC1);
    cv::Mat down_right = upper_left;
    cv::Mat upper_right = -upper_left;
    cv::Mat down_left = upper_right;
    cv::Mat kernel_diag;
    cv::Mat temp1, temp2;
    cv::hconcat(upper_left, upper_right, temp1);
    cv::hconcat(down_left, down_right, temp2);
    cv::vconcat(temp1, temp2, kernel_diag);

    // compute haar feature ground truth
    double haar_feature_hor_ground_truth = temp.dot(kernel_hor);
    double haar_feature_ver_ground_truth = temp.dot(kernel_ver);
    double haar_feature_diag_ground_truth = temp.dot(kernel_diag);

    // compute haar feature via integral img
    img.convertTo(img, CV_64FC1);
    cv::Mat haar_feature_mat_hor = cv::Mat(img.size(), CV_64FC1, std::numeric_limits<double>::infinity());
    cv::Mat haar_feature_mat_ver = cv::Mat(img.size(), CV_64FC1, std::numeric_limits<double>::infinity());
    cv::Mat haar_feature_mat_diag = cv::Mat(img.size(), CV_64FC1, std::numeric_limits<double>::infinity());
    cv::Mat img_integral = make_integral_img(img);
    cv::Mat vis_inte_img = get_float_mat_vis_img(img_integral);
    cv::imshow("Integral Img", vis_inte_img);
    cv::waitKey(0);

    // compute hor haar feature
    for (int r = 0; r < img_integral.rows - height; r++) {
        for (int c = 0; c < img_integral.cols - width; c++) {
            haar_feature_mat_hor.at<double>(r, c) =
                compute_rect_integral_img_from_ul(img_integral, r, c, width, height / 2) -
                compute_rect_integral_img_from_ul(img_integral, r + height / 2, c, width, height / 2);

            haar_feature_mat_ver.at<double>(r, c) =
                compute_rect_integral_img_from_ul(img_integral, r, c, width / 2, height) -
                compute_rect_integral_img_from_ul(img_integral, r, c + width / 2, width / 2, height);

            haar_feature_mat_diag.at<double>(r, c) =
                compute_rect_integral_img_from_ul(img_integral, r, c, width / 2, height / 2) +
                compute_rect_integral_img_from_ul(img_integral, r + height / 2, c + width / 2, width / 2, height / 2) -
                compute_rect_integral_img_from_ul(img_integral, r + height / 2, c, width / 2, height / 2) -
                compute_rect_integral_img_from_ul(img_integral, r, c + width / 2, width / 2, height / 2);
        }
    }
    double min_dist = std::numeric_limits<double>::max();
    std::vector<cv::Point2i> detection_position;
    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            double dist = std::pow(haar_feature_hor_ground_truth - haar_feature_mat_hor.at<double>(r, c), 2) +
                          std::pow(haar_feature_ver_ground_truth - haar_feature_mat_ver.at<double>(r, c), 2) +
                          std::pow(haar_feature_diag_ground_truth - haar_feature_mat_diag.at<double>(r, c), 2);
            if (dist < min_dist) {
                min_dist = dist;
                if (dist < 1000000000.0) {
                    detection_position.emplace_back(c, r);
                }
            }
        }
    }

    // cv::Rect2i detection(detection_position.x,detection_position.y,width,height);
    cv::Mat vis;
    for (auto rect_pos : detection_position) {
        vis = draw_bounding_box_vis_image(img, rect_pos.x, rect_pos.y, width, height);
    }
    vis = get_float_mat_vis_img(vis);
    cv::imshow("Detection Result", vis);
    cv::waitKey(0);
    return 0;
}