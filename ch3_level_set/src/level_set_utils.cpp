#include "level_set_utils.h"
#include "display.h"
#include <opencv2/imgproc.hpp>

cv::Mat computer_div_delta_map(const SDFMap& sdf_map) {
    cv::Mat phi = sdf_map.map_;
    cv::Mat d_phi_dx, d_phi_dy;
    cv::Sobel(phi, d_phi_dx, CV_64F, 1, 0, 3);
    cv::Sobel(phi, d_phi_dy, CV_64F, 0, 1, 3);
    cv::Mat d_phi_dxx, d_phi_dxy, d_phi_dyy;
    cv::Sobel(d_phi_dx, d_phi_dxx, CV_64F, 1, 0, 3);
    cv::Sobel(d_phi_dx, d_phi_dxy, CV_64F, 0, 1, 3);
    cv::Sobel(d_phi_dy, d_phi_dyy, CV_64F, 0, 1, 3);
    cv::Mat pow_d_phi_dx, pow_d_phi_dy, pow_phi_denominator;
    cv::pow(d_phi_dx, 2, pow_d_phi_dx);
    cv::pow(d_phi_dy, 2, pow_d_phi_dy);
    cv::pow(pow_d_phi_dx + pow_d_phi_dy, -2 / 3, pow_phi_denominator);
    return (d_phi_dxx.mul(pow_d_phi_dy) -
            2 * d_phi_dx.mul(d_phi_dy.mul(d_phi_dxy)) +
            d_phi_dyy.mul(pow_d_phi_dx))
        .mul(pow_phi_denominator);
}

cv::Mat compute_div_delta_map(const SDFMap& sdf_map) {
    cv::Mat phi = sdf_map.map_;
    cv::Mat phi_grad_mag = compute_mat_grad_magnitude(phi);
    cv::Mat d_phi_dx, d_phi_dy;
    cv::Sobel(phi, d_phi_dx, CV_64F, 1, 0, 3);
    d_phi_dx.mul(1.0 /
                 (phi_grad_mag + 1e-8 * cv::Mat::ones(phi.size(), phi.type())));
    cv::Sobel(phi, d_phi_dy, CV_64F, 0, 1, 3);
    d_phi_dy.mul(1.0 /
                 (phi_grad_mag + 1e-8 * cv::Mat::ones(phi.size(), phi.type())));

    cv::Sobel(d_phi_dx, d_phi_dx, CV_64F, 1, 0, 3);
    cv::Sobel(d_phi_dy, d_phi_dy, CV_64F, 0, 1, 3);
    // std::cout << d_phi_dy + d_phi_dx << '\n';
    // return 3e4 * cv::Mat::ones(phi.size(), phi.type());
    return (d_phi_dx + d_phi_dy);
}
inline double heaviside(double z, double eps) {
    return 0.5 * (1 + M_2_PI * atan2(z, eps));
}

inline double dirac(double z, double eps = 1.0) {
    return eps * M_1_PI / (std::pow(eps, 2.0) + std::pow(z, 2.0));
}

cv::Mat heaviside(const SDFMap& sdf_map, double eps) {
    cv::Mat phi = sdf_map.map_;
    cv::Mat result(phi.size(), phi.type());
    std::transform(phi.begin<double>(), phi.end<double>(),
                   result.begin<double>(),
                   [=](double z) { return heaviside(z, eps); });

    return result;
}

cv::Mat complementary_heaviside(const SDFMap& sdf_map, double eps) {
    cv::Mat h_phi = heaviside(sdf_map, eps);
    return cv::Mat::ones(h_phi.size(), h_phi.type()) - h_phi;
}

cv::Mat dirac(const SDFMap& sdf_map, double eps) {
    cv::Mat phi = sdf_map.map_;
    cv::Mat result(phi.size(), phi.type());
    std::transform(phi.begin<double>(), phi.end<double>(),
                   result.begin<double>(),
                   [=](double z) { return dirac(z, eps); });
    return result;
}

double compute_length_energy(const SDFMap& sdf_map, double eps) {
    cv::Mat heaviside_map = heaviside(sdf_map, eps);
    return cv::sum(heaviside_map)[0];
}
cv::Mat compute_laplacian_map(const SDFMap& sdf_map) {
    cv::Mat result;
    cv::Laplacian(sdf_map.map_, result, CV_64F, 3, cv::BORDER_REPLICATE);
    return result;
}

cv::Mat compute_square_diff(cv::Mat img1, cv::Mat img2) {
    cv::Mat diff = img1 - img2;
    cv::Mat square_diff;
    cv::pow(diff, 2.0f, square_diff);
    return square_diff;
}

cv::Mat compute_derivative_data_term(const SDFMap& sdf_map,
                                     cv::Mat original_image,
                                     double weight_foreground,
                                     double weight_background,
                                     double center_foreground,
                                     double center_background, double eps) {
    // todo homework
    cv::Mat e_foreground = compute_square_diff(
        original_image,
        center_foreground *
            cv::Mat::ones(original_image.size(), original_image.type()));
    cv::Mat e_background = compute_square_diff(
        original_image,
        center_background *
            cv::Mat::ones(original_image.size(), original_image.type()));

    cv::Mat vis;
    cv::Mat vis_dirac = get_float_mat_vis_img(dirac(sdf_map, eps));
    cv::Mat vis_e_foreground = get_float_mat_vis_img(e_foreground);
    cv::Mat vis_e_background = get_float_mat_vis_img(e_background);

    cv::hconcat(vis_dirac, vis_e_foreground, vis);
    cv::hconcat(vis, vis_e_background, vis);
    disp_image(vis, "top: dirac, mid : e_foregroud, down: e_backgroud", 1);

    return dirac(sdf_map, eps)
        .mul((weight_foreground * e_foreground -
              weight_background * e_background));
}

cv::Mat compute_derivative_length_term(const SDFMap& sdf_map, double eps) {
    cv::Mat div = compute_div_delta_map(sdf_map);
    cv::Mat vis_div = get_float_mat_vis_img(div);
    disp_image(vis_div, "div", 1);
    cv::Mat dirac_map = dirac(sdf_map, eps);
    return dirac_map.mul(div);
}

cv::Mat compute_derivative_gradient_term(const SDFMap& sdf_map) {
    cv::Mat laplacian_map_result = compute_laplacian_map(sdf_map);
    return laplacian_map_result - compute_div_delta_map(sdf_map);
}

cv::Mat gaussian_kernel(int size, double sigma) {
    assert(size % 2 == 1);
    cv::Point center((size - 1) / 2, (size - 1) / 2);
    cv::Mat result = cv::Mat::zeros(cv::Size(size, size), CV_64F);
    for (int r = 0; r < size; r++) {
        for (int c = 0; c < size; c++) {
            result.at<double>(r, c) =
                (M_1_PI * 0.5 / (sigma * sigma)) *
                exp(-(pow(r - center.x, 2) + pow(c - center.y, 2)) /
                    (2 * sigma * sigma));
        }
    }
    return result / cv::sum(result)[0];
}

// todo untested function
double compute_center(cv::Mat img, const SDFMap& sdf_map, double eps,
                      bool is_background) {
    if (is_background == true) {
        return cv::sum(img.mul(heaviside(sdf_map, eps)))[0] /
               cv::sum(heaviside(sdf_map, eps))[0];
    } else {
        return cv::sum(img.mul(complementary_heaviside(sdf_map, eps)))[0] /
               cv::sum(complementary_heaviside(sdf_map, eps))[0];
    }
}

cv::Mat compute_mat_grad_magnitude(cv::Mat mat) {
    cv::Mat mat_dev_x, mat_dev_y;
    cv::Sobel(mat, mat_dev_x, CV_64F, 1, 0, 3);
    cv::Sobel(mat, mat_dev_y, CV_64F, 0, 1, 3);
    cv::Mat mat_grad_square =
        (mat_dev_x.mul(mat_dev_x) + mat_dev_y.mul(mat_dev_y));
    cv::Mat mat_grad_magnitude;
    cv::sqrt(mat_grad_square, mat_grad_magnitude);
    return mat_grad_magnitude;
}

double compute_data_term_energy(const SDFMap& sdf_map, cv::Mat original_image,
                                double weight_foreground,
                                double weight_background,
                                double center_foreground,
                                double center_background, double eps) {
    cv::Mat e_foreground = compute_square_diff(
        original_image,
        center_foreground *
            cv::Mat::ones(original_image.size(), original_image.type()));
    cv::Mat e_background = compute_square_diff(
        original_image,
        center_background *
            cv::Mat::ones(original_image.size(), original_image.type()));

    return weight_foreground *
               cv::sum(e_foreground.mul(heaviside(sdf_map, eps)))[0] +
           weight_background * cv::sum(e_background.mul(
                                   complementary_heaviside(sdf_map, eps)))[0];
}

double compute_length_term_energy(const SDFMap& sdf_map, double eps) {
    cv::Mat heaviside_map = heaviside(sdf_map, eps);
    cv::Mat heaviside_map_grad_magnitude =
        compute_mat_grad_magnitude(heaviside_map);
    return cv::sum(heaviside_map_grad_magnitude)[0];
}

// todo compute mat grad magnitude should be friend of sdf_map
double compute_gradient_preserve_energy(const SDFMap& sdf_map) {
    cv::Mat sdf_map_grad_magnitude = compute_mat_grad_magnitude(sdf_map.map_);
    cv::Mat sdf_map_grad_preserve = compute_square_diff(
        sdf_map_grad_magnitude, cv::Mat::ones(sdf_map_grad_magnitude.size(),
                                              sdf_map_grad_magnitude.type()));

    return cv::sum(sdf_map_grad_preserve * 0.5)[0];
}

cv::Mat do_sobel_y(cv::Mat input) {
    if (input.channels() != 1) {
        cv::cvtColor(input, input, cv::COLOR_BGR2GRAY);
    }

    if (input.type() != CV_64FC1) {
        input.convertTo(input, CV_64FC1);
    }
    cv::Mat output(input.size(), input.type());
    for (int r = 0; r < input.rows; r++) {
        for (int c = 0; c < input.cols; c++) {
            int r_uhs = -1;
            r_uhs = std::min(std::max(0, r_uhs), input.rows - 1);
            int r_dhs = 1;
            r_dhs = std::min(std::max(0, r_dhs), input.rows - 1);
            output.at<double>(r, c) = 0.5 * (input.at<double>(r + r_dhs, c) -
                                             input.at<double>(r + r_uhs, c));
        }
    }

    return output;
}