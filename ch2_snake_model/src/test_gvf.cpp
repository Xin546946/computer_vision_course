#include "display.h"
#include "gvf.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

cv::Mat external_force_image(cv::Mat img, const float w_line,
                             const float w_edge, const float w_term,
                             const float sigma) {
    cv::Mat Ix, Iy, Ixx, Iyy, Ixy;
    // Gaussian blur
    cv::Mat image;
    cv::GaussianBlur(img, image, cv::Size(3, 3), 10, 10);
    image.convertTo(image, CV_32F);

    // calculate the image first derivative w.r.t. x, y as well as the second
    // derivative w.r.t xx,yy,xy
    cv::Sobel(image, Ix, CV_32F, 1, 0, 3);
    cv::Sobel(image, Iy, CV_32F, 0, 1, 3);
    cv::Sobel(Ix, Ixx, CV_32F, 1, 0, 3);
    cv::Sobel(Iy, Iyy, CV_32F, 0, 1, 3);
    cv::Sobel(Ix, Ixy, CV_32F, 0, 1, 3);

    cv::Mat e_line, e_edge, e_term, e_extern;
    cv::GaussianBlur(image, e_line, cv::Size(3, 3), 1, 1);
    // Calculate (Iyy.*Ix.*Ix - 2*Ixy.*Ix.*Iy + Ixx.*Iy.*Iy)./
    // ((1+Ix.*Ix+Iy.*Iy)**1.5) tmp1 = Iyy.*Ix.*Ix tmp2 = 2*Ixy.*Ix.*Iy tmp3 =
    // Ixx.*Iy.*Iy tmp4 = (1+Ix.*Ix+Iy.*Iy)
    cv::Mat tmp1, tmp2, tmp3, tmp4, tmp5, tmp6;
    cv::multiply(Ix, Ix, tmp1);
    cv::multiply(Ix, Ix, tmp4);
    cv::multiply(tmp1, Iyy, tmp1);
    cv::multiply(Ix, Iy, tmp2);
    cv::multiply(tmp2, -2 * Ixy, tmp2);
    cv::multiply(Iy, Iy, tmp3);
    cv::multiply(Iy, Iy, tmp5);
    cv::multiply(tmp3, Ixx, tmp3);

    e_term = tmp1 + tmp2 + tmp3;

    // cout << one_matrix << endl;
    cv::Mat tes = tmp5 + tmp4 + 1;
    // cout << tes << endl;

    pow(tmp5 + tmp4 + 1, 1.5, tmp6);

    divide(e_term, tmp6, e_term);

    sqrt((tmp5 + tmp4), e_edge);

    e_extern = w_line * e_line - w_edge * e_edge - w_term * e_term;
    disp_image(e_extern, "e_extern", 0);
    // std::cout << e_extern << std::endl;
    return e_extern;
}

int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], 0);
    std::cout << "Image channels: " << img.channels() << std::endl;
<<<<<<< HEAD
    cv::GaussianBlur(img, img, cv::Size(7, 7), 7, 7);
    disp_image(img, "gauss", 0);
    cv::Mat grad_x_original, grad_y_original;

    cv::Sobel(img, grad_x_original, CV_32F, 1, 0, 3);
    cv::Sobel(img, grad_y_original, CV_32F, 0, 1, 3);
=======
    cv::Mat external_energy = external_force_image(img, 0.04, 2, 0.01, 10);

    cv::GaussianBlur(external_energy, external_energy, cv::Size(3, 3), 20, 20);
    cv::Mat grad_x_original, grad_y_original;
>>>>>>> 60ff58ecd7aef526f2a39818e4c093130b5cd31c

    cv::Sobel(external_energy, grad_x_original, CV_32F, 1, 0, 3);
    cv::Sobel(external_energy, grad_y_original, CV_32F, 0, 1, 3);
    grad_x_original = -grad_x_original * 2 * 9;
    grad_y_original = -grad_y_original * 2 * 9;
    ParamGVF param_gvf(0, 10, 1e-10);
    GVF gvf(grad_x_original, grad_y_original, param_gvf);
<<<<<<< HEAD
    gvf.run(2000);
    std::vector<cv::Mat> gvf_result = gvf.get_result_gvf();

    cv::normalize(gvf_result[0], gvf_result[0], -1, 1, cv::NORM_MINMAX);
    // disp_image(gvf_result[0], "gvf[0]", 0);
    // std::cout << gvf_result[0] << std::endl;

    cv::normalize(gvf_result[1], gvf_result[1], -1, 1, cv::NORM_MINMAX);
    // disp_image(gvf_result[1], "gvf[1]", 0);
=======
    gvf.run(10000);
    std::vector<cv::Mat> gvf_result = gvf.get_result_gvf();

    cv::normalize(gvf_result[0], gvf_result[0], -1, 1, cv::NORM_MINMAX);
    disp_image(gvf_result[0], "gvf[0]", 0);
    // std::cout << gvf_result[0] << std::endl;

    cv::normalize(gvf_result[1], gvf_result[1], -1, 1, cv::NORM_MINMAX);
    disp_image(gvf_result[1], "gvf[1]", 0);
>>>>>>> 60ff58ecd7aef526f2a39818e4c093130b5cd31c

    cv::Mat gvf_show = img.clone();
    cv::cvtColor(gvf_show, gvf_show, CV_GRAY2RGB);
    // cv::normalize(gvf_show, gvf_show);
    cv::Scalar color(0, 255, 0);
<<<<<<< HEAD
    draw_optical_flow(gvf_result[0], gvf_result[1], gvf_show, 8, 5, color);
    disp_image(gvf_show, "gvf", 0);
=======
    draw_optical_flow(gvf_result[0], gvf_result[1], gvf_show, 5, 10, color);
    disp_image(gvf_show, "gvf");
>>>>>>> 60ff58ecd7aef526f2a39818e4c093130b5cd31c
    return 0;
}