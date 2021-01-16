#include "cf_tracker.h"
// #include "bounding_box.h"
#include "math_utils.h"
#include "opencv_utils.h"

cv::Mat get_sub_image(cv::Mat img, BoundingBox bbox) {
    return get_sub_image_around(img, std::round(bbox.center().x), std::round(bbox.center().y), std::round(bbox.width()),
                                std::round(bbox.height()));
}
cv::Mat compute_mag_fft(cv::Mat complex_img) {
    cv::Mat planes[2];
    cv::split(complex_img, planes);                  // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
    cv::magnitude(planes[0], planes[1], planes[0]);  // planes[0] = magnitude
    cv::Mat mag_img = planes[0];
    mag_img += cv::Scalar::all(1);  // switch to logarithmic scale
    log(mag_img, mag_img);
    // crop the spectrum, if it has an odd number of rows or columns
    mag_img = mag_img(cv::Rect(0, 0, mag_img.cols & -2, mag_img.rows & -2));
    // rearrange the quadrants of Fourier image  so that the origin is at the image center
    // Shift the origin center of Fourier img
    int cx = mag_img.cols / 2;
    int cy = mag_img.rows / 2;
    cv::Mat q0(mag_img, cv::Rect(0, 0, cx, cy));    // Top-Left - Create a ROI per quadrant
    cv::Mat q2(mag_img, cv::Rect(0, cy, cx, cy));   // Bottom-Left
    cv::Mat q1(mag_img, cv::Rect(cx, 0, cx, cy));   // Top-Right
    cv::Mat q3(mag_img, cv::Rect(cx, cy, cx, cy));  // Bottom-Right
    cv::Mat tmp;                                    // swap quadrants (Top-Left with Bottom-Right)
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);
    q1.copyTo(tmp);  // swap quadrant (Top-Right with Bottom-Left)
    q2.copyTo(q1);
    tmp.copyTo(q2);
    cv::normalize(mag_img, mag_img, 0, 1, cv::NORM_MINMAX);  // Transform the matrix with float values into a
    // viewable image form (float between values 0 and 1).
    return mag_img;
}

cv::Mat compute_fft(cv::Mat img) {
    int rows_dft = cv::getOptimalDFTSize(img.rows);
    int cols_dft = cv::getOptimalDFTSize(img.cols);
    cv::Mat padded;  // expand input image to optimal size
    cv::copyMakeBorder(img, padded, 0, rows_dft - img.rows, 0, cols_dft - img.cols, cv::BORDER_CONSTANT,
                       cv::Scalar::all(0));
    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32FC1)};
    cv::Mat complex_img;
    cv::merge(planes, 2, complex_img);  // Add to the expanded another plane with zeros
    cv::dft(complex_img, complex_img);  // this way the result may fit in the source matrix
    return complex_img;
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

cv::Mat compute_ifft(cv::Mat complex_img) {
    cv::Mat ifft_img;
    cv::dft(complex_img, ifft_img, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT);
    cv::normalize(ifft_img, ifft_img, 0, 1, CV_MINMAX);
    return ifft_img;
}

CFTracker::CFTracker(const BoundingBox& bbox)
    : bbox_(bbox),
      RESPONSE_(cv::Mat::zeros(cv::Size(2 * bbox_.width(), 2 * bbox_.height()), CV_64FC2)),
      KERNEL_A_(RESPONSE_.clone()),
      KERNEL_B_(RESPONSE_.clone()),
      KERNEL_(RESPONSE_.clone()) {
    // int rows_dft = cv::getOptimalDFTSize(RESPONSE_.rows);
    // int cols_dft = cv::getOptimalDFTSize(RESPONSE_.cols);
    // RESPONSE_ = cv::Mat::zeros(cv::Size(cols_dft, rows_dft), CV_64FC2);
    // KERNEL_A_ = cv::Mat::zeros(cv::Size(cols_dft, rows_dft), CV_64FC2);
    // KERNEL_B_ = cv::Mat::zeros(cv::Size(cols_dft, rows_dft), CV_64FC2);
    // KERNEL_ = cv::Mat::zeros(cv::Size(cols_dft, rows_dft), CV_64FC2);
    // std::cout << "@@@@@@@@" << KERNEL_A_.type() << " " << RESPONSE_.type() << '\n';
}

cv::Mat generate_response(cv::Mat sub_img_from_roi, BoundingBox bbox_origin) {
    assert(sub_img_from_roi.type() == CV_64FC1);
    BoundingBox bbox_coordinate(bbox_origin.width() / 2.0f, bbox_origin.height() / 2.0f, bbox_origin.width(),
                                bbox_origin.height());  //! search box and tracking box
    cv::Mat G = cv::Mat::zeros(sub_img_from_roi.size(), sub_img_from_roi.type());
    // cv::Mat vis_sub_img;  //= get_float_mat_vis_img(sub_img_from_roi);
    // sub_img_from_roi.convertTo(vis_sub_img, cv::COLOR_GRAY2BGR);
    // cv::Mat vis_img_bbox =
    //     draw_bounding_box_vis_image(vis_sub_img, bbox_coordinate.top_left().x, bbox_coordinate.top_left().y,
    //                                 bbox_coordinate.width(), bbox_coordinate.height());
    // cv::imshow("vis sub image", vis_img_bbox);
    // cv::waitKey(0);
    // BoundingBox bbox = 2 * bbox_origin;
    put_val_around(255.0, G, bbox_coordinate.center().x, bbox_coordinate.center().y, 3, 3);
    // for (int r = bbox_coordinate.center().y - 1; r <= bbox_coordinate.center().y + 1; r++) {
    //     for (int c = bbox_coordinate.center().x - 1; c <= bbox_coordinate.center().x + 1; c++) {
    //         G.at<double>(r, c) = 255.0;
    //     }
    // }
    cv::Mat gaussian_G;
    cv::GaussianBlur(G, gaussian_G, cv::Size(5, 5), 2.0, 2.0);
    // std::cout << gaussian_G << '\n';
    assert(gaussian_G.type() == CV_64FC1);
    // cv::Mat vis_gauss = get_float_mat_vis_img(gaussian_G);
    // cv::imshow("Gaussian G", vis_gauss);
    // cv::waitKey(0);
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

void CFTracker::preprocessing(cv::Mat& img) {
    img.convertTo(img, CV_64FC1);
    cv::log(img + 1.0f, img);

    cv::Scalar mean, std_dev;
    cv::meanStdDev(img, mean, std_dev);

    img = (img - mean[0]) / (std_dev[0] + 1e-6);
}

void CFTracker::train_H(cv::Mat img) {
    cv::Mat img_64f;
    img.convertTo(img_64f, CV_64FC1);  // change img to CV_64F for calculation
    // cv::Mat vis_img;                   // = get_float_mat_vis_img(img_64f);  // use this func for visualization
    // cv::cvtColor(img, vis_img, CV_GRAY2BGR);
    // cv::imshow("vis_img", vis_img);
    // cv::waitKey(0);
    // cv::Mat vis_bbox_img = draw_bounding_box_vis_image(
    //     vis_img, (2.0f * bbox_).top_left().x, (2.0f * bbox_).top_left().y, bbox_.width() * 2.0f, bbox_.height()
    //     * 2.0f);
    // cv::imshow("test for bigger bbox", vis_bbox_img);
    // cv::waitKey(0);
    cv::Mat roi_img = get_sub_image(img_64f, 2.0 * bbox_);  //! search box
    if (roi_img.rows != bbox_.height() || roi_img.cols != bbox_.width()) {
        KERNEL_A_ = get_sub_image_from_ul(KERNEL_A_, 0, 0, roi_img.cols, roi_img.rows);
        KERNEL_B_ = get_sub_image_from_ul(KERNEL_B_, 0, 0, roi_img.cols, roi_img.rows);
    }
    assert(roi_img.type() == CV_64FC1);
    // std::cout << "roi img is CV_64FC1 type" << '\n';

    cv::Mat response = generate_response(roi_img, bbox_);
    // test
    // cv::Mat vis_response = get_float_mat_vis_img(response);
    // cv::imshow("response", vis_response);
    // cv::waitKey(0);
    // cv::Mat RESPONSE;
    cv::dft(response, RESPONSE_, cv::DFT_COMPLEX_OUTPUT);
    for (int i = 0; i < 8; i++) {
        preprocessing(roi_img);
        cv::Mat img_train = cmpute_rand_affine_transformation(roi_img);
        // cv::Mat vis_img_train = get_float_mat_vis_img(img_train);
        // cv::imshow("vis_img_train", vis_img_train);
        // cv::waitKey(0);
        cv::Mat IMG_TRAIN;  // = compute_fft(img_train);
        cv::dft(img_train, IMG_TRAIN, cv::DFT_COMPLEX_OUTPUT);

        cv::Mat NOM, DEN;
        cv::mulSpectrums(RESPONSE_, IMG_TRAIN, NOM, cv::DFT_ROWS, true);

        cv::mulSpectrums(IMG_TRAIN, IMG_TRAIN, DEN, cv::DFT_ROWS, true);
        KERNEL_A_ = KERNEL_A_ + NOM;
        KERNEL_B_ = KERNEL_B_ + DEN;
    }
    KERNEL_ = div_fft(KERNEL_A_, KERNEL_B_ + 0.1);
    KERNEL_ /= 8.f;
    cv::Mat kernel;  // = compute_ifft(KERNEL_);
    cv::idft(KERNEL_, kernel, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
    cv::Mat vis_kernel = get_float_mat_vis_img(kernel);
    cv::imshow("kernel", vis_kernel);
    cv::waitKey(0);
    std::cout << KERNEL_.channels() << '\n';
    //! test for fft(img) * kernel == fft(response)
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
    // if (img.size() != KERNEL_.size()) {
    //     //! todo
    //     img = get_sub_image_from_ul(img, 0, 0, KERNEL_.cols, KERNEL_.cols);
    //     // KERNEL_ = get_sub_image_from_ul(KERNEL_, 0, 0, img.cols, img.rows);
    //     // RESPONSE_ = get_sub_image_from_ul(RESPONSE_, 0, 0, img.cols, img.rows);
    //     // std::cout << " Tracking terminate." << '\n';
    //     // std::exit(0);
    // }
    preprocessing(img);
    // calculate G using previoud H and current img
    cv::Mat IMG;  //= compute_fft(img);
    cv::dft(img, IMG, cv::DFT_COMPLEX_OUTPUT);
    cv::mulSpectrums(IMG, KERNEL_, RESPONSE_, cv::DFT_ROWS, true);

    cv::Mat KERNEL_A_NEW, KERNEL_B_NEW;

    cv::mulSpectrums(RESPONSE_, IMG, KERNEL_A_NEW, cv::DFT_ROWS, true);
    cv::mulSpectrums(IMG, IMG, KERNEL_B_NEW, cv::DFT_ROWS, true);

    KERNEL_A_ = KERNEL_A_ * (1 - rate_) + KERNEL_A_NEW * rate_;
    KERNEL_B_ = KERNEL_B_ * (1 - rate_) + KERNEL_B_NEW * rate_;

    KERNEL_ = div_fft(KERNEL_A_, KERNEL_B_);
}

void CFTracker::update_bbox(cv::Mat img) {
    back_up_state();
    cv::Mat IMG;  //! = compute_fft(img);

    cv::dft(img, IMG, cv::DFT_COMPLEX_OUTPUT);
    cv::mulSpectrums(IMG, KERNEL_, RESPONSE_, cv::DFT_ROWS, true);
    cv::Mat response;  //! = compute_ifft(RESPONSE_);
    cv::idft(RESPONSE_, response, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);

    // update center position
    double max_val;
    cv::Point max_location;
    cv::minMaxLoc(response, 0, &max_val, 0, &max_location);
    cv::Mat response_local = get_sub_image_around(response, max_location.x, max_location.y, 11, 11);
    delta_x_ = (float(max_location.x) - response.size().width / 2);
    delta_y_ = (float(max_location.y) - response.size().height / 2);
    float PSR;
    cv::Scalar mean, std_dev;
    cv::meanStdDev(response_local, mean, std_dev);
    PSR = (max_val - mean[0]) / (std_dev[0] + 1e-6);
    std::cout << "@@@PSR: " << PSR << '\n';
    if (PSR < 5.7) {
        delta_x_ = delta_x_prev_ * 0.5 + delta_x_ * 0.5;
        delta_y_ = delta_y_prev_ * 0.5 + delta_y_ * 0.5;
    }
    bbox_.move(delta_x_, delta_y_);
}

void CFTracker::back_up_state() {
    delta_x_prev_ = delta_x_;
    delta_y_prev_ = delta_y_;
}

void CFTracker::visualize(cv::Mat img) {
    // cv::Mat vis_img = get_float_mat_vis_img(img);
    // vis_img.convertTo(vis_img, CV_8UC1);
    cv::Mat img_color;
    cv::cvtColor(img, img_color, CV_GRAY2BGR);
    cv::Mat vis_bbox =
        draw_bounding_box_vis_image(img_color, bbox_.top_left().x, bbox_.top_left().y, bbox_.width(), bbox_.height());
    cv::imshow("Tracking result", vis_bbox);
    cv::waitKey(10);
}

void CFTracker::process(const std::vector<cv::Mat>& video) {
    // cv::Mat sub_img = get_sub_image(video[0], 2.0 * bbox_);
    train_H(video[0]);
    for (cv::Mat frame : video) {
        cv::Mat frame_64f;
        frame.convertTo(frame_64f, CV_64FC1);
        // int top = KERNEL_.rows * 3;
        // int bottom = top;
        // int left = KERNEL_.cols * 3;
        // int right = left;
        // cv::copyMakeBorder(frame_64f, frame_64f, top, bottom, left, right, cv::BORDER_CONSTANT);
        cv::Mat sub_frame = get_sub_image(frame_64f, 2.0f * bbox_);

        // std::cout << "sub frame size" << sub_frame.size() << '\n';
        // cv::Mat vis_sub_frame = get_float_mat_vis_img(sub_frame);
        // cv::imshow("vis sub frame", vis_sub_frame);
        // cv::waitKey(1);
        visualize(frame);
        // if ((2.0f * bbox_).is_out_of_img(frame)) {
        //     std::cout << "Tracking terminates." << '\n';
        //     std::exit(0);
        // }
        update_bbox(sub_frame);
        update_H(sub_frame);
    }
}