#include "bounding_box.h"
#include "display.h"
#include "math_utils.h"
#include "opencv_utils.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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

cv::Mat compute_ifft(cv::Mat complex_img) {
    cv::Mat ifft_img;
    cv::dft(complex_img, ifft_img, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT);
    cv::normalize(ifft_img, ifft_img, 0, 1, CV_MINMAX);
    return ifft_img;
}

cv::Mat get_sub_img_from_roi(cv::Mat img, BoundingBox bbox, double ratio_width, double ratio_height) {
    BoundingBox bigger_bbx = 2.0 * bbox;
    cv::Mat sub_img_from_roi = get_sub_image_from_ul(img, bigger_bbx.top_left().x, bigger_bbx.top_left().y,
                                                     bigger_bbx.width(), bigger_bbx.height());
    return sub_img_from_roi;
}

cv::Mat get_output_response(cv::Mat sub_img_from_roi, BoundingBox bbox) {
    cv::Mat G = cv::Mat::zeros(sub_img_from_roi.size(), sub_img_from_roi.type());
    G.at<double>(std::floor(bbox.height() / 2), std::floor(bbox.width() / 2)) = 255.0;
    cv::Mat gaussian_G;
    cv::GaussianBlur(G, gaussian_G, cv::Size(11, 11), 3.0, 3.0);
    return gaussian_G;
}

//! no tested! Q: how to test??
cv::Mat div_fft(cv::Mat fft_img1, cv::Mat fft_img2) {
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
    cv::divide(fft_img2_2_channels[0].mul(fft_img1_2_channels[1]) - fft_img1_2_channels[0].mul(fft_img2_2_channels[1]),
               denom, im);

    cv::Mat temp[2] = {re, im};
    cv::Mat result;
    cv::merge(temp, 2, result);
    return result;
}
cv::Mat get_sub_image(cv::Mat img, BoundingBox bbox) {
    return get_sub_image_around(img, bbox.center().x, bbox.center().y, bbox.width(), bbox.height());
}
cv::Mat cmpute_rand_affine_transformation(cv::Mat img) {
    std::cout << "@@@@@@ pi is: " << M_PI << '\n';
    float angle = generate_random_data(float(-M_PI / 10), float(M_PI / 10));
    // std::cout << angle << '\n';
    float disturb = generate_random_data(0.8f, 1.2f);
    double cos_angle = cos(angle);
    double sin_angle = sin(angle);
    cv::Mat_<double> rotation_matrix(2, 3);
    rotation_matrix << disturb * cos_angle, -disturb * sin_angle, 0, disturb * sin_angle, disturb * cos_angle, 0;

    // cv::Mat_<double> translation_vec(2, 1);
    // translation_vec << img.cols / 2, img.rows / 2;
    // rotation_matrix.col(2) = translation_vec - (rotation_matrix.colRange(0, 2)) * translation_vec;
    std::cout << "Test function of colRange(0,1)" << rotation_matrix.colRange(0, 2) << '\n';
    std::cout << rotation_matrix << '\n';
    cv::Mat wraped;
    cv::warpAffine(img, wraped, rotation_matrix, img.size(), cv::BORDER_REFLECT);
    return wraped;
}

cv::Mat compute_response(cv::Mat sub_img_from_roi, BoundingBox bbox_origin) {
    assert(sub_img_from_roi.size() == cv::Size(bbox_origin.width(), bbox_origin.height()));
    cv::Mat G = cv::Mat::zeros(sub_img_from_roi.size(), sub_img_from_roi.type());
    BoundingBox bbox = 2 * bbox_origin;
    G.at<double>(std::floor(bbox.height() / 2), std::floor(bbox.width() / 2)) = 255.0;
    cv::Mat gaussian_G;
    cv::GaussianBlur(G, gaussian_G, cv::Size(11, 11), 3.0, 3.0);
    return gaussian_G;
}

cv::Mat train_H(cv::Mat img, BoundingBox& bbox) {
    cv::Mat roi_img = get_sub_image(img, bbox);
    cv::imshow("roi img", roi_img);
    cv::waitKey(0);
    cv::Mat response = compute_response(roi_img, bbox);
    cv::Mat RESPONSE = compute_fft(response);
    // cv::dft(response, RESPONSE, cv::DFT_COMPLEX_OUTPUT);
    cv::Mat kernel_A = cv::Mat::zeros(RESPONSE.size(), RESPONSE.type());
    cv::Mat kernel_B = cv::Mat::zeros(RESPONSE.size(), RESPONSE.type());
    for (int i = 0; i < 8; i++) {
        cv::Mat img_train = cmpute_rand_affine_transformation(roi_img);
        cv::imshow("affine_transform", img_train);
        cv::waitKey(0);
        cv::Mat IMG_TRAIN = compute_fft(img_train);
        // cv::dft(img_train, IMG_TRAIN, cv::DFT_COMPLEX_OUTPUT);

        cv::Mat nom, den;
        cv::mulSpectrums(RESPONSE, IMG_TRAIN, nom, cv::DFT_ROWS, true);

        cv::mulSpectrums(IMG_TRAIN, IMG_TRAIN, den, cv::DFT_ROWS, true);
        std::cout << "size of nom: " << nom.channels() << '\n';
        std::cout << "size of den: " << den.channels() << '\n';
        std::cout << "size of kernel: " << kernel_A.size() << '\n';
        kernel_A += nom;
        kernel_B += den;
    }
    return div_fft(kernel_A, kernel_B);
}

int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::Mat temp = read_img(argv[2], cv::IMREAD_GRAYSCALE);

    cv::Point2i init_upper_left = template_matching(img, temp);
    cv::Point2f init_center = cv::Point2f(init_upper_left.x + temp.cols / 2.0f, init_upper_left.y + temp.rows / 2.0f);

    BoundingBox bbox(init_upper_left.x, init_upper_left.y, temp.cols, temp.rows);
    cv::Mat test = get_sub_image(img, bbox);
    cv::imshow("test", test);
    cv::waitKey(0);
    cv::Mat vis;
    cv::cvtColor(img, vis, CV_GRAY2BGR);
    draw_bounding_box_vis_image(vis, bbox.top_left().x, bbox.top_left().y, bbox.width(), bbox.height());
    BoundingBox bigger_bbox = 2.0 * bbox;
    draw_bounding_box_vis_image(vis, bigger_bbox.top_left().x, bigger_bbox.top_left().y, bigger_bbox.width(),
                                bigger_bbox.height());
    cv::imshow("BoundingBox", vis);
    cv::waitKey(0);
    // img.convertTo(img, CV_64FC1);
    cv::Mat sub_img_from_roi = get_sub_image_from_ul(img, bigger_bbox.top_left().x, bigger_bbox.top_left().y,
                                                     bigger_bbox.width(), bigger_bbox.height());
    cv::imshow("sub img from roi", sub_img_from_roi);
    cv::waitKey(0);
    cv::Mat G = cv::Mat::zeros(sub_img_from_roi.size(), sub_img_from_roi.type());
    assert(bigger_bbox.center() == bbox.center());
    bigger_bbox.print_bbox_info();
    bbox.print_bbox_info();
    test = get_sub_image(img, bbox);
    cv::imshow("test", test);
    cv::waitKey(0);
    cv::circle(vis, bigger_bbox.center(), 3, cv::Scalar(255, 0, 0));
    cv::imshow("BoundingBox", vis);
    cv::waitKey(0);
    G.at<double>(std::floor(bigger_bbox.height() / 2), std::floor(bigger_bbox.width() / 2)) = 255.0;
    cv::Mat gaussian_G;
    cv::GaussianBlur(G, gaussian_G, cv::Size(11, 11), 3.0, 3.0);
    cv::Mat vis_G = get_float_mat_vis_img(gaussian_G);
    // std::cout << gaussian_G << '\n';
    cv::imshow("Response", vis_G);
    cv::waitKey(0);

    cv::Mat H = train_H(img, bigger_bbox);
    cv::Mat SUB_IMG_FROM_ROI;
    cv::dft(sub_img_from_roi, SUB_IMG_FROM_ROI, cv::DFT_COMPLEX_OUTPUT);
    cv::Mat RESULT;
    cv::mulSpectrums(H, SUB_IMG_FROM_ROI, RESULT, cv::DFT_REAL_OUTPUT, true);
    assert(H.size() == SUB_IMG_FROM_ROI.size());
    cv::Mat result;
    cv::idft(RESULT, result, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
    std::cout << result << '\n';
    cv::imshow("result", result);
    cv::waitKey(0);

    return 0;
}