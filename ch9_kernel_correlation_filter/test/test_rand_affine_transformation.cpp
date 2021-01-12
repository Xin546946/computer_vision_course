#include "bounding_box.h"
#include "math_utils.h"
#include "opencv_utils.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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

int main(int argc, char** argv) {
    cv::Mat img = cv::imread("/home/kit/computer_vision_course/ch9_kernel_correlation_filter/dataset/data25/1.jpg",
                             cv::IMREAD_GRAYSCALE);
    cv::Mat temp =
        read_img("/home/kit/computer_vision_course/ch9_kernel_correlation_filter/dataset/data25/template/1.png",
                 cv::IMREAD_GRAYSCALE);

    cv::Point2i init_upper_left = template_matching(img, temp);
    cv::Point2f init_center = cv::Point2f(init_upper_left.x + temp.cols / 2.0f, init_upper_left.y + temp.rows / 2.0f);

    BoundingBox bbox(init_upper_left.x, init_upper_left.y, temp.cols, temp.rows);
    BoundingBox bigger_bbox = 3.0 * bbox;
    cv::Mat sub_img = get_sub_image(img, bigger_bbox);
    cv::imshow("sub img", sub_img);
    cv::waitKey(0);
    cv::Mat wraped = cmpute_rand_affine_transformation(sub_img);

    cv::Mat vis_bbox =
        draw_bounding_box_vis_image(img, bbox.top_left().x, bbox.top_left().y, bbox.width(), bbox.height());
    cv::imshow("warp_img", vis_bbox);
    cv::waitKey(0);
    return 0;
}