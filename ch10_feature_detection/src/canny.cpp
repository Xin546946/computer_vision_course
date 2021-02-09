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
    cv::normalize(amplitudes, amplitudes, 0, 1, cv::NORM_MINMAX);
    cv::Mat vis = get_float_mat_vis_img(amplitudes);
    cv::imshow("amplitude", vis);
    cv::waitKey(0);

    // gradient non-maxima-suppression
    int nms_size = 10;
    cv::Mat img_grad_x_normalized(img_grad_x.mul(1 / amplitudes));
    cv::Mat img_grad_y_normalized(img_grad_y.mul(1 / amplitudes));
    int max, r_grad, c_grad;
    for (int r = 0; r < amplitudes.rows; r++) {
        for (int c = 0; c < amplitudes.cols; c++) {
            double amplitude = amplitudes.at<double>(r, c);
            max = std::max(std::abs(img_grad_x_normalized.at<double>(r, c)),
                           std::abs(img_grad_y_normalized.at<double>(r, c)));
            for (int k = -nms_size; k < nms_size + 1; k++) {
                r_grad = static_cast<int>(r + k * (img_grad_x_normalized.at<double>(r, c) / max));
                c_grad = static_cast<int>(c + k * (img_grad_y_normalized.at<double>(r, c) / max));
                if (r_grad < amplitudes.rows && r_grad >= 0 && c_grad < amplitudes.cols && c_grad >= 0) {
                    if (amplitude < amplitudes.at<double>(r_grad, c_grad)) {
                        amplitudes.at<double>(r, c) = 0.0;
                        break;
                    }
                }
            }
        }
    }

    cv::imshow("after NMS", get_float_mat_vis_img(amplitudes));
    cv::waitKey(0);

    cv::Mat img_threshold_1, img_threshold_2;
    double threshold_1(0.08), threshold_2(0.26);
    cv::threshold(amplitudes, img_threshold_1, threshold_1, 1, cv::THRESH_BINARY);
    cv::threshold(amplitudes, img_threshold_2, threshold_2, 1, cv::THRESH_BINARY);
    cv::imshow("threshold 1", img_threshold_1);
    cv::waitKey(0);
    cv::imshow("threshold 2", img_threshold_2);
    cv::waitKey(0);

    // step3: double threshold to get edge

    int window_size_thres = 3;
    for (int r = 0; r < img_threshold_1.rows; r++) {
        for (int c = 0; c < img_threshold_1.cols; c++) {
            // std::cout << img_threshold_2.at<double>(r, c) << '\n';
            if (double(img_threshold_2.at<double>(r, c)) > 0.0) {
                for (int r_win = -window_size_thres; r_win < window_size_thres; r_win++) {
                    for (int c_win = -window_size_thres; c_win < window_size_thres; c_win++) {
                        if (double(img_threshold_1.at<double>(r + r_win, c + c_win)) > 0.0) {
                            result_.at<double>(r + r_win, c + c_win) = 255.0;
                        }
                    }
                }
            }
        }
    }
    // std::cout << result_ << '\n';
}