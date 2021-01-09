#include "bounding_box.h"
#include "display.h"
#include "opencv_utils.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

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
    BoundingBox bigger_bbx = bbox.set_larger_roi(ratio_width, ratio_height);
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

int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::Mat temp = read_img(argv[2], cv::IMREAD_GRAYSCALE);

    cv::Point2i init_upper_left = template_matching(img, temp);
    cv::Point2f init_center = cv::Point2f(init_upper_left.x + temp.cols / 2.0f, init_upper_left.y + temp.rows / 2.0f);

    BoundingBox bbox(init_upper_left.x, init_upper_left.y, temp.cols, temp.rows);
    cv::Mat vis;
    cv::cvtColor(img, vis, CV_GRAY2BGR);
    draw_bounding_box_vis_image(vis, bbox.top_left().x, bbox.top_left().y, bbox.width(), bbox.height());
    BoundingBox bigger_bbx = bbox.set_larger_roi(2, 2);
    draw_bounding_box_vis_image(vis, bigger_bbx.top_left().x, bigger_bbx.top_left().y, bigger_bbx.width(),
                                bigger_bbx.height());
    cv::imshow("BoundingBox", vis);
    cv::waitKey(0);
    img.convertTo(img, CV_64FC1);
    cv::Mat sub_img_from_roi = get_sub_image_from_ul(img, bigger_bbx.top_left().x, bigger_bbx.top_left().y,
                                                     bigger_bbx.width(), bigger_bbx.height());
    cv::Mat G = cv::Mat::zeros(sub_img_from_roi.size(), sub_img_from_roi.type());
    assert(bigger_bbx.center() == bbox.center());
    cv::circle(vis, bigger_bbx.center(), 3, cv::Scalar(255, 0, 0));
    cv::imshow("BoundingBox", vis);
    cv::waitKey(0);
    G.at<double>(std::floor(bigger_bbx.height() / 2), std::floor(bigger_bbx.width() / 2)) = 255.0;
    cv::Mat gaussian_G;
    cv::GaussianBlur(G, gaussian_G, cv::Size(11, 11), 3.0, 3.0);
    cv::Mat vis_G = get_float_mat_vis_img(gaussian_G);
    // std::cout << gaussian_G << '\n';
    cv::imshow("Response", vis_G);
    cv::waitKey(0);

    return 0;
}