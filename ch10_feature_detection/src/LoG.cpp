#include "LoG.h"
#include "opencv_utils.h"

void LoG::run() {
    // step 1: 用Gaussian函数对图像进行平滑，抑制噪声
    cv::GaussianBlur(img_, result_, param_gaussian_.size_, param_gaussian_.sigma_x_, param_gaussian_.sigma_y_,
                     cv::BORDER_DEFAULT);
    cv::imshow("Gaussian Blur", result_);
    cv::waitKey(0);
    cv::Mat result_gray;
    cv::cvtColor(result_, result_gray, cv::COLOR_BGR2GRAY);
    cv::imshow("gray value", result_gray);
    cv::waitKey(0);
    // step 2: 对经过平滑的图像使用Laplacian算子
    cv::Mat lap_dist;
    cv::Laplacian(result_gray, lap_dist, param_laplacian_.ddepth_, param_laplacian_.kernel_size_,
                  param_laplacian_.scale_, param_laplacian_.delta_, cv::BORDER_DEFAULT);

    // step 3: 二值化图像（一般以0为阈值）
    cv::convertScaleAbs(lap_dist, lap_dist);
    // cv::Mat vis_lap = get_float_mat_vis_img(lap_dist);
    cv::imshow("Laplacian", lap_dist);
    cv::waitKey(0);
    cv::Mat thres_dist;
    cv::threshold(lap_dist, thres_dist, param_thres_.threshold_, param_thres_.max_val_, param_thres_.threshold_type_);

    cv::imshow("threshold to binary img", thres_dist);
    cv::waitKey(0);

    // step 4: 使用形态学方法（膨胀腐蚀）得到图像边缘
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
    cv::Mat erosion_dist, dilation_dist;
    cv::dilate(thres_dist, dilation_dist, element, cv::Point(-1, -1), 1);   // 膨胀
    cv::erode(dilation_dist, erosion_dist, element, cv::Point(-1, -1), 2);  // 腐蚀
    cv::imshow("Result", erosion_dist);
    cv::waitKey(0);
}