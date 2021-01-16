/**
______________________________________________________________________
*********************************************************************
* @brief This file is developed for the course of ShenLan XueYuan:
* Fundamental implementations of Computer Vision
* all rights preserved
* @author Xin Jin, Zhaoran Wu
* @contact: xinjin1109@gmail.com, zhaoran.wu1@gmail.com
*
______________________________________________________________________
*********************************************************************
**/
#include "cf_tracker.h"
// #include "bounding_box.h"
#include "math_utils.h"
#include "opencv_utils.h"

cv::Mat get_sub_image(cv::Mat img, BoundingBox bbox) {
    return get_sub_image_around(img, bbox.center().x, bbox.center().y, bbox.width(), bbox.height());
}

cv::Mat div_fft(const cv::Mat& fft_img1, const cv::Mat& fft_img2) {
    cv::Mat fft_img1_2_channels[2], fft_img2_2_channels[2];
    cv::split(fft_img1, fft_img1_2_channels);  // fft_img1 = a + ib
    cv::split(fft_img2, fft_img2_2_channels);  // fft_img2 = c + id

    // compute c**2 + d**2
    cv::Mat denom =
        fft_img2_2_channels[0].mul(fft_img2_2_channels[0]) + fft_img2_2_channels[1].mul(fft_img2_2_channels[1]);

    // compute (ac+bd)/(cc+dd)
    cv::Mat re, im;
    cv::divide(fft_img1_2_channels[0].mul(fft_img2_2_channels[0]) + fft_img1_2_channels[1].mul(fft_img2_2_channels[1]),
               denom, re, 1.0);

    // compute (cb - ad)/(cc+dd)
    cv::divide(fft_img2_2_channels[0].mul(fft_img1_2_channels[1]) - fft_img1_2_channels[0].mul(fft_img2_2_channels[1]),
               denom, im, -1.0);

    cv::Mat temp[2] = {re, im};
    cv::Mat result;
    cv::merge(temp, 2, result);
    return result;
}

CFTracker::CFTracker(const BoundingBox& bbox)
    : bbox_(bbox),
      RESPONSE_(cv::Mat::zeros(cv::Size(2 * bbox_.width(), 2 * bbox_.height()), CV_64FC2)),
      KERNEL_A_(RESPONSE_.clone()),
      KERNEL_B_(RESPONSE_.clone()),
      KERNEL_(RESPONSE_.clone()) {
}

cv::Mat cmpute_rand_affine_transformation(cv::Mat img) {
    // todo compute random affine transformation
    // todo hints: cv::warpAffine
}

void CFTracker::train_init_kernel(cv::Mat img) {
    // todo train initial kernel
    // todo hints: cv::dft(XXX), cv::idft(XXX), cv::mulSpectrums(XXX)
}

void CFTracker::update_kernel(cv::Mat img) {
    // todo update kernel
    // todo hints: div_fft(XXX)
}

void CFTracker::update_bbox(cv::Mat img) {
    // todo update bbox
    // todo hints: minmaxLoc
}

void CFTracker::visualize(cv::Mat img) {
    cv::Mat img_color;
    cv::cvtColor(img, img_color, CV_GRAY2BGR);
    cv::Mat vis_bbox =
        draw_bounding_box_vis_image(img_color, bbox_.top_left().x, bbox_.top_left().y, bbox_.width(), bbox_.height());
    cv::imshow("Tracking result", vis_bbox);
    cv::waitKey(10);
}

void CFTracker::process(const std::vector<cv::Mat>& video) {
    train_init_kernel(video[0]);
    for (cv::Mat frame : video) {
        cv::Mat frame_64f;
        frame.convertTo(frame_64f, CV_64FC1);
        cv::Mat sub_frame = get_sub_image(frame_64f, 2.0f * bbox_);

        visualize(frame);

        update_kernel(sub_frame);

        update_bbox(sub_frame);
    }
}