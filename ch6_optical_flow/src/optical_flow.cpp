#include "optical_flow.h"
#include "opencv_utils.h"

OpticalFlow::OpticalFlow(cv::Mat img1, cv::Mat img2, std::vector<cv::Point2f> prev_fps,
                         std::vector<cv::Point2f> curr_fps, std::vector<uchar> status, cv::Size2i win_size)
    : img1_(img1), img2_(img2), fps_(prev_fps) {
    for (cv::Point2f fp : fps_) {
        curr_fps.push_back(compute_flow_in_window(fp, win_size));
        update_status();
    }
}

cv::Point2f OpticalFlow::compute_flow_in_window(cv::Point2f feature_point, cv::Size win_size) {
    cv::Mat img_grad_x = do_sobel(img1_, 0);
    cv::Mat img_grad_y = do_sobel(img1_, 1);
    cv::Mat img_grad_x_roi =
        get_sub_image_around(img_grad_x, feature_point.x, feature_point.y, win_size.width, win_size.height);
    cv::Mat img_grad_y_roi =
        get_sub_image_around(img_grad_y, feature_point.x, feature_point.y, win_size.width, win_size.height);
    double Ixy = img_grad_x_roi.dot(img_grad_y_roi);
    cv::Matx22d A(img_grad_x_roi.dot(img_grad_x_roi), Ixy, Ixy, img_grad_y_roi.dot(img_grad_y_roi));
}