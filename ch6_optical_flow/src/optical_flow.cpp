#include "optical_flow.h"
#include "opencv_utils.h"

OpticalFlow::OpticalFlow(cv::Mat img1, cv::Mat img2, const std::vector<cv::Point2f>& prev_fps, cv::Size2i win_size)
    : img1_(img1), img2_(img2), fps_(prev_fps), win_size_(win_size) {
}

std::vector<cv::Point2f> OpticalFlow::compute_curr_fps() {
    std::vector<cv::Point2f> curr_fps;
    for (cv::Point2f fp : fps_) {
        curr_fps.push_back(compute_flow_in_window(fp));
        update_status();
    }
}

cv::Point2f OpticalFlow::compute_flow_in_window(cv::Point2f feature_point) {
    cv::Mat img_grad_x = do_sobel(img1_, 0);
    cv::Mat img_grad_y = do_sobel(img1_, 1);
    cv::Mat img_grad_x_roi =
        get_sub_image_around(img_grad_x, feature_point.x, feature_point.y, win_size_.width, win_size_.height);
    cv::Mat img_grad_y_roi =
        get_sub_image_around(img_grad_y, feature_point.x, feature_point.y, win_size_.width, win_size_.height);
    float Ixy = img_grad_x_roi.dot(img_grad_y_roi);
    cv::Matx22f A(img_grad_x_roi.dot(img_grad_x_roi), Ixy, Ixy, img_grad_y_roi.dot(img_grad_y_roi));
}