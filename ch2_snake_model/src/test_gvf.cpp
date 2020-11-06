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
    cv::GaussianBlur(img, image, cv::Size(3, 3), 3, 3);
    image.convertTo(image, CV_32F);

    // calculate the image first derivative w.r.t. x, y as well as the second
    // derivative w.r.t xx,yy,xy
    cv::Sobel(image, Ix, CV_32F, 1, 0,
              3);  // Compute gradient of blurred edge map
    cv::Sobel(image, Iy, CV_32F, 0, 1, 3);
    cv::Sobel(Ix, Ixx, CV_32F, 1, 0,
              3);  // Compute gradient of blurred edge map
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

    cv::pow(tmp5 + tmp4 + 1, 1.5, tmp6);

    cv::divide(e_term, tmp6, e_term);

    cv::sqrt((tmp5 + tmp4), e_edge);

    e_extern = w_line * e_line - w_edge * e_edge - w_term * e_term;
    /*namedWindow("e_line", WINDOW_AUTOSIZE);
    imshow("e_line", e_line);
    waitKey(0);
    namedWindow("e_edge", WINDOW_AUTOSIZE);
    imshow("e_edge", e_edge);
    waitKey(0);
    namedWindow("e_term", WINDOW_AUTOSIZE);
    imshow("e_term", e_term);
    waitKey(0);

    namedWindow("e_extern",WINDOW_AUTOSIZE);
    imshow("e_extern",e_extern);
    waitKey(0); */
    return e_extern;
}

int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], 0);
    // cv::Mat external_energy_img = external_force_image(img, 0.04, 2, 0.01,
    // 10);
    std::cout << "Image channels: " << img.channels() << std::endl;
    cv::GaussianBlur(img, img, cv::Size(3, 3), 3, 3);

    cv::Mat grad_x_original, grad_y_original;
    cv::Sobel(img, grad_x_original, CV_32F, 1, 0, 3);
    cv::Sobel(img, grad_y_original, CV_32F, 0, 1, 3);
    // cv::GaussianBlur(external_energy_img, external_energy_img, cv::Size(7,
    // 7),
    //                  7, 7);

    // cv::GaussianBlur(grad_y_original, grad_y_original, cv::Size(7, 7), 7, 7);
    // cv::Sobel(external_energy_img, grad_x_original, CV_32F, 1, 0, 3);
    // cv::Sobel(external_energy_img, grad_y_original, CV_32F, 0, 1, 3);
    // grad_x_original = -grad_x_original * 2 * 20 * 20;
    // grad_y_original = -grad_y_original * 2 * 20 * 20;
    // Parameter tune:(offer good result)
    // 1. Eext = wline*e_line + wedge*edge+wterm*term :
    // param_gvf(1e122,1,1e-12)
    // 2. grad||grad(img)||
    // TODO NOT sensible to smooth term?? if smooth term = 0, energy also
    // decreases
    ParamGVF param_gvf(1e4, 21, 1e-5);  // mu , sigma, init step size

    GVF gvf(grad_x_original, grad_y_original, param_gvf);
    gvf.run(1e5);  // parameter: max_iteration
    return 0;
}