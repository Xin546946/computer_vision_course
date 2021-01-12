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
    // compute (cb - ad)/(cc+dd)
    cv::Mat re, im;
    cv::divide(fft_img1_2_channels[0].mul(fft_img2_2_channels[0]) + fft_img1_2_channels[1].mul(fft_img2_2_channels[1]),
               denom, re);
    cv::divide(fft_img2_2_channels[0].mul(fft_img1_2_channels[1]) + fft_img1_2_channels[0].mul(fft_img2_2_channels[1]),
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

cv::Mat compute_response(cv::Mat sub_img_from_roi, BoundingBox bbox_origin) {
    assert(sub_img_from_roi.type() == CV_64FC1);
    BoundingBox bbox_coordinate(bbox_origin.width() / 2.0f, bbox_origin.height() / 2.0f, bbox_origin.width(),
                                bbox_origin.height());
    cv::Mat G = cv::Mat::zeros(sub_img_from_roi.size(), sub_img_from_roi.type());
    cv::Mat vis_sub_img;  //= get_float_mat_vis_img(sub_img_from_roi);
    sub_img_from_roi.convertTo(vis_sub_img, cv::COLOR_GRAY2BGR);
    cv::Mat vis_img_bbox =
        draw_bounding_box_vis_image(vis_sub_img, bbox_coordinate.top_left().x, bbox_coordinate.top_left().y,
                                    bbox_coordinate.width(), bbox_coordinate.height());
    cv::imshow("vis sub image", vis_img_bbox);
    cv::waitKey(0);
    // BoundingBox bbox = 2 * bbox_origin;
    put_val_around(255.0, G, bbox_coordinate.center().x, bbox_coordinate.center().y, 3, 3);
    // for (int r = bbox_coordinate.center().y - 1; r <= bbox_coordinate.center().y + 1; r++) {
    //     for (int c = bbox_coordinate.center().x - 1; c <= bbox_coordinate.center().x + 1; c++) {
    //         G.at<double>(r, c) = 255.0;
    //     }
    // }
    cv::Mat gaussian_G;
    cv::GaussianBlur(G, gaussian_G, cv::Size(5, 5), 10.0, 10.0);
    // std::cout << gaussian_G << '\n';
    assert(gaussian_G.type() == CV_64FC1);
    cv::Mat vis_gauss = get_float_mat_vis_img(gaussian_G);
    cv::imshow("Gaussian G", vis_gauss);
    cv::waitKey(0);
    return gaussian_G;
}

cv::Mat cmpute_rand_affine_transformation(cv::Mat img) {
    // std::cout << "@@@@@@ pi is: " << M_PI << '\n';
    float angle = generate_random_data(-0.1f, 0.1f);
    // std::cout << angle << '\n';
    // float disturb = generate_random_data(0.8f, 1.2f);
    double cos_angle = cos(angle);
    double sin_angle = sin(angle);
    cv::Mat_<double> rotation_matrix(2, 3);
    rotation_matrix << generate_random_data(-0.1f, 0.1f) + cos_angle, -generate_random_data(-0.1f, 0.1f) + sin_angle, 0,
        generate_random_data(-0.1f, 0.1f) + sin_angle, generate_random_data(-0.1f, 0.1f) + cos_angle, 0;

    cv::Mat_<double> translation_vec(2, 1);
    translation_vec << img.cols / 2, img.rows / 2;
    rotation_matrix.col(2) = translation_vec - (rotation_matrix.colRange(0, 2)) * translation_vec;
    // std::cout << "Test function of colRange(0,1)" << rotation_matrix.colRange(0, 2) << '\n';
    // std::cout << rotation_matrix << '\n';
    cv::Mat wraped;
    cv::warpAffine(img, wraped, rotation_matrix, img.size(), cv::BORDER_REFLECT);
    return wraped;
}

void CFTracker::train_H(cv::Mat img) {
    cv::Mat img_64f;
    img.convertTo(img_64f, CV_64FC1);  // change img to CV_64F for calculation
    cv::Mat vis_img;                   // = get_float_mat_vis_img(img_64f);  // use this func for visualization
    img_64f.convertTo(vis_img, CV_GRAY2BGR);
    // cv::imshow("vis_img", vis_img);
    // cv::waitKey(0);
    cv::Mat vis_bbox_img = draw_bounding_box_vis_image(
        vis_img, (2.0f * bbox_).top_left().x, (2.0f * bbox_).top_left().y, bbox_.width() * 2.0f, bbox_.height() * 2.0f);
    cv::imshow("test for bigger bbox", vis_bbox_img);
    cv::waitKey(0);
    cv::Mat roi_img = get_sub_image(img_64f, 2.0 * bbox_);
    assert(roi_img.type() == CV_64FC1);
    std::cout << "roi img is CV_64FC1 type" << '\n';

    cv::Mat response = compute_response(roi_img, bbox_);
    // // test
    // cv::Mat vis_response = get_float_mat_vis_img(response);
    // cv::imshow("response", vis_response);
    // cv::waitKey(0);
    // cv::Mat RESPONSE;
    cv::dft(response, RESPONSE_, cv::DFT_COMPLEX_OUTPUT);
    for (int i = 0; i < 8; i++) {
        cv::Mat img_train = cmpute_rand_affine_transformation(roi_img);
        cv::Mat vis_img_train = get_float_mat_vis_img(img_train);
        cv::imshow("vis_img_train", vis_img_train);
        cv::waitKey(0);
        cv::Mat IMG_TRAIN;
        cv::dft(img_train, IMG_TRAIN, cv::DFT_COMPLEX_OUTPUT);

        cv::Mat NOM, DEN;
        cv::mulSpectrums(RESPONSE_, IMG_TRAIN, NOM, cv::DFT_ROWS, true);

        cv::mulSpectrums(IMG_TRAIN, IMG_TRAIN, DEN, cv::DFT_ROWS, true);
        KERNEL_A_ += NOM;
        KERNEL_B_ += DEN;
    }
    KERNEL_ = div_fft(KERNEL_A_, KERNEL_B_);
    std::cout << KERNEL_.channels() << '\n';
    // test for fft(img) * kernel == fft(response)
    cv::Mat ROI_IMG;
    cv::dft(roi_img, ROI_IMG, cv::DFT_COMPLEX_OUTPUT);
    cv::Mat TEST_RESPONSE, test_response;
    std::cout << "IMG size: " << ROI_IMG.size() << '\n' << "KERNEL size: " << KERNEL_.size() << '\n';
    cv::mulSpectrums(ROI_IMG, KERNEL_, TEST_RESPONSE, cv::DFT_ROWS, true);
    cv::idft(TEST_RESPONSE, test_response, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
    std::cout << "Type of test response: " << test_response.type() << '\n';
    cv::Mat vis_test_response = get_float_mat_vis_img(test_response);
    cv::imshow("test_response", vis_test_response);
    cv::waitKey(0);
}

void CFTracker::update_H(cv::Mat img) {
    // cv::Mat roi_img = get_sub_image(img, bbox_);
    cv::Mat KERNEL_A_NEW, KERNEL_B_NEW, IMG;
    cv::dft(img, IMG, cv::DFT_COMPLEX_OUTPUT);
    cv::mulSpectrums(RESPONSE_, IMG, KERNEL_A_NEW, cv::DFT_ROWS, true);
    cv::mulSpectrums(IMG, IMG, KERNEL_B_NEW, cv::DFT_ROWS, true);

    KERNEL_A_ = KERNEL_A_ * (1 - rate_) + KERNEL_A_NEW * rate_;
    KERNEL_B_ = KERNEL_B_ * (1 - rate_) + KERNEL_B_NEW * rate_;

    KERNEL_ = div_fft(KERNEL_A_, KERNEL_B_);
}

void CFTracker::update_bbox(cv::Mat img) {
    cv::Mat IMG;
    cv::dft(img, IMG, cv::DFT_COMPLEX_OUTPUT);
    cv::mulSpectrums(IMG, KERNEL_, RESPONSE_, cv::DFT_ROWS, true);
    cv::Mat response;
    cv::idft(RESPONSE_, response, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);

    // update center position
    double max_val;
    cv::Point max_location;
    cv::minMaxLoc(response, 0, &max_val, 0, &max_location);
    float delta_x = (float(max_location.x) - response.size().width / 2);
    float delta_y = (float(max_location.y) - response.size().height / 2);
    bbox_.move(delta_x, delta_y);
}

void CFTracker::process(const std::vector<cv::Mat>& video) {
    // cv::Mat sub_img = get_sub_image(video[0], 2.0 * bbox_);
    train_H(video[0]);
    for (cv::Mat frame : video) {
        cv::Mat sub_frame = get_sub_image(frame, bbox_);
        update_H(sub_frame);
        update_bbox(sub_frame);
        cv::Mat vis_bbox =
            draw_bounding_box_vis_image(frame, bbox_.top_left().x, bbox_.top_left().y, bbox_.width(), bbox_.height());
        cv::imshow("Tracking result", vis_bbox);
        cv::waitKey(0);
    }
}