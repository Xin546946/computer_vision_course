#include "canny.h"

void Canny::run() {
    // step1: Gaussianblur
    cv::GaussianBlur(img_64f_, img_64f_, cv::Size(3, 3), 3.0, 3.0);

    // Compute sobel x and sobel y to get amplitude of edge
    cv::Mat img_grad_x, img_grad_y;
    cv::Sobel(img_64f_, img_grad_x, -1, 1, 0, 3);
    cv::Sobel(img_64f_, img_grad_y, -1, 0, 1, 3);
    cv::Mat amplitudes = img_grad_x.mul(img_grad_x) + img_grad_y.mul(img_grad_y);
    cv::sqrt(amplitudes, amplitudes);
    cv::Mat vis = get_float_mat_vis_img(amplitudes);
    cv::imshow("amplitude", vis);
    cv::waitKey(0);

    // non-maxima-suppression
    int nms_size = 3;
    double threshold_1 = 45.0;
    double threshold_2 = 150.0;
    cv::Mat img_threshold_1(cv::Mat::zeros(img_.size(), CV_64FC1)),
        img_threshold_2(cv::Mat::zeros(img_.size(), CV_64FC1));
    int border = (nms_size - 1) / 2;
    cv::copyMakeBorder(img_64f_, img_64f_, border, border, border, border, cv::BORDER_CONSTANT, 200);
    // todo nms should be along the gradient
    for (int r = border; r < amplitudes.rows - border; r++) {
        for (int c = border; c < amplitudes.cols - border; c++) {
            double amplitude = amplitudes.at<double>(r, c);

            // if (amplitude > threshold) {
            bool local_max = true;
            for (int r_win = -border; r_win < border; r_win++) {
                for (int c_win = -border; c_win < border; c_win++) {
                    if (r_win == 0 && c_win == 0) {
                        continue;
                    }
                    local_max = local_max && (amplitude > amplitudes.at<double>(r + r_win, c + c_win));
                }
            }
            if (local_max) {
                if (amplitude > threshold_1) {
                    img_threshold_1.at<double>(r - border, c - border) = 255.0;
                }
                if (amplitude > threshold_2) {
                    img_threshold_2.at<double>(r - border, c - border) = 255.0;
                }
            }
            // }
        }
    }
    // std::cout << img_threshold_2 << '\n';
    cv::Mat vis_thres1 = get_float_mat_vis_img(img_threshold_1);
    cv::Mat vis_thres2 = get_float_mat_vis_img(img_threshold_2);
    cv::imshow("threshold 1", img_threshold_1);
    cv::waitKey(0);
    cv::imshow("threshold 2", img_threshold_2);
    cv::waitKey(0);
    // step3: double threshold to get edge
    result_ = img_threshold_2.clone();
    cv::imshow("result_", result_);
    cv::waitKey(0);
    int window_size_thres = 5;
    int border_thres = (window_size_thres - 1) / 2;
    cv::copyMakeBorder(img_threshold_1, img_threshold_1, border_thres, border_thres, border_thres, border_thres,
                       cv::BORDER_CONSTANT, 0);
    cv::copyMakeBorder(img_threshold_1, img_threshold_1, border_thres, border_thres, border_thres, border_thres,
                       cv::BORDER_CONSTANT, 0);
    for (int r = border_thres; r < img_threshold_1.rows - border_thres; r++) {
        for (int c = border_thres; c < img_threshold_1.cols - border_thres; c++) {
            // std::cout << img_threshold_2.at<double>(r, c) << '\n';
            if (double(img_threshold_2.at<double>(r, c)) == 255.0) {
                for (int r_win = -border_thres; r_win < border_thres; r_win++) {
                    for (int c_win = -border_thres; c_win < border_thres; c_win++) {
                        if (double(img_threshold_1.at<double>(r + r_win, c + c_win)) == 255.0) {
                            result_.at<double>(r + r_win - border_thres, c + c_win - border_thres) = 255.0;
                        }
                    }
                }
            }
        }
    }
}