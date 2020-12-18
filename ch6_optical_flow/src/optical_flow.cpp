#include "optical_flow.h"
#include "opencv_utils.h"
#include <opencv2/core/core.hpp>

OpticalFlow::OpticalFlow(cv::Mat img1, cv::Mat img2, const std::vector<cv::Point2f>& prev_fps, cv::Size2i win_size)
    : img1_(img1), img2_(img2), fps_(prev_fps), curr_fps_(prev_fps), win_size_(win_size), status_(prev_fps.size()) {
    process_optical_flow();
}

void OpticalFlow::process_optical_flow() {
    cv::Mat grad_x_img = do_sobel(cv::Mat_<float>(img1_), 0);
    cv::Mat grad_y_img = do_sobel(cv::Mat_<float>(img1_), 1);
    // std::vector<cv::Point2f> curr_fps;
    // for (cv::Point2f fp : fps_) {
    for (int i = 0; i < fps_.size(); i++) {
        cv::Mat grad_x_img_roi =
            get_sub_image_around(grad_x_img, fps_[i].x, fps_[i].y, win_size_.width, win_size_.height);
        cv::Mat grad_y_img_roi =
            get_sub_image_around(grad_y_img, fps_[i].x, fps_[i].y, win_size_.width, win_size_.height);
        cv::Matx22f A = compute_A(grad_x_img_roi, grad_y_img_roi);
        std::cout << "A is : " << '\n';
        std::cout << A << '\n';
        status_[i] = update_status(A);
        std::cout << status_[i];
        cv::Mat roi_img1 = get_sub_image_around(img1_, fps_[i].x, fps_[i].y, win_size_.width, win_size_.height);
        cv::Mat roi_img2 = get_sub_image_around(img2_, fps_[i].x, fps_[i].y, win_size_.width, win_size_.height);
        cv::Matx21f b = compute_b(roi_img1, roi_img2, grad_x_img_roi, grad_y_img_roi);
        cv::Matx21f optical_flow = A.solve(b, cv::DECOMP_CHOLESKY);
        curr_fps_[i] = cv::Point2f(optical_flow(0, 0), optical_flow(0, 1));
    }
    // curr_fps.push_back(compute_flow_in_window(fp, grad_x_img, grad_y_img));
}

cv::Matx22f OpticalFlow::compute_A(cv::Mat grad_x, cv::Mat grad_y) {
    assert(grad_x.size() == grad_y.size());
    float Ixy = grad_x.dot(grad_y);
    return cv::Matx22f(grad_x.dot(grad_x), Ixy, Ixy, grad_y.dot(grad_y));
}

cv::Matx21f OpticalFlow::compute_b(cv::Mat prev_img, cv::Mat curr_img, cv::Mat grad_x, cv::Mat grad_y) {
    assert(prev_img.size() == curr_img.size());

    assert(prev_img.type() == curr_img.type() && grad_x.type() == grad_y.type() && prev_img.type() == grad_x.type());
    cv::Mat diff_img_roi = curr_img - prev_img;
    return cv::Matx21f(grad_x.dot(diff_img_roi), grad_y.dot(diff_img_roi));
}

uchar OpticalFlow::update_status(cv::Matx22f matrix) {
    double det = cv::determinant(matrix);
    double trace = cv::trace(matrix);
    float kappa = 1.0f;
    return (det - kappa * trace * trace) > 0;
}

std::vector<uchar> OpticalFlow::get_status() const {
    return status_;
}

std::vector<cv::Point2f> OpticalFlow::get_curr_fps() const {
    return curr_fps_;
}