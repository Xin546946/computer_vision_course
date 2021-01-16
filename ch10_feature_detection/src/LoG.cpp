#include "LoG.h"
#include "opencv_utils.h"

void LoG::run() {
    cv::GaussianBlur(img_, result_, param_gaussian_.size_, param_gaussian_.sigma_x_, param_gaussian_.sigma_y_,
                     cv::BORDER_DEFAULT);
    cv::imshow("Gaussian Blur", result_);
    cv::waitKey(0);
    cv::Mat result_gray;
    cv::cvtColor(result_, result_gray, cv::COLOR_BGR2GRAY);
    cv::imshow("gray value", result_gray);
    cv::waitKey(0);
    cv::Mat lap_dist;
    // cv::Laplacian(result_gray, lap_dist, param_laplacian_.ddepth_, param_laplacian_.kernel_size_,
    //               param_laplacian_.scale_, param_laplacian_.delta_, cv::BORDER_DEFAULT);
    cv::Laplacian(result_gray, lap_dist, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT);

    cv::convertScaleAbs(lap_dist, lap_dist);
    cv::Mat vis_lap = get_float_mat_vis_img(lap_dist);
    std::cout << lap_dist << '\n';
    cv::imshow("Laplacian", lap_dist);
    cv::waitKey(0);
    cv::Mat thres_dist;
    cv::threshold(lap_dist, thres_dist, param_thres_.threshold_, param_thres_.max_val_, param_thres_.threshold_type_);

    cv::imshow("threshold to binary img", thres_dist);
    cv::waitKey(0);
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1), cv::Point(0, 0));
    cv::Mat erosion_dist, dilation_dist;
    cv::erode(thres_dist, erosion_dist, element, cv::Point(-1, -1), 3);
    cv::dilate(erosion_dist, dilation_dist, element, cv::Point(-1, -1), 3);
    cv::imshow("Result", dilation_dist);
    cv::waitKey(0);
}