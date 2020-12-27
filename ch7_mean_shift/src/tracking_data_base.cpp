#include "tracking_data_base.h"
#include "data_base.h"
#include "display.h"
#include "memory"
#include "opencv_utils.h"
#include "visualizer.h"
#include <algorithm>
#include <chrono>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>

cv::Mat compute_back_projection(cv::Mat img, cv::Mat hist_temp, cv::Mat hist_candidate);

cv::Point2f TrackerDataBase::get_object_center() const {
    return bbox_.center();
}

void TrackerDataBase::init_mass_center() {
}

void TrackerDataBase::update_mass_center() {
    last_bbox_ = bbox_;
    int num_bin = 10;
    double sigma = 10;
    cv::Mat back_proj_weight = compute_back_projection_weight(num_bin, sigma);

    cv::Point2f mean_shift = compute_mean_shift(back_proj_weight, sigma);

    bbox_.move_center_to(mean_shift.x, mean_shift.y);
}

bool TrackerDataBase::is_convergent() {
    return cv::norm(bbox_.center() - last_bbox_.center()) < 1e-4;
}

void TrackerDataBase::set_obj_predicted_initial_center(cv::Point2f pos) {
    //! update bounding box using this pose
    //! todo make sure the bbox is inside the image
    if (pos.x + temp_64f_.cols > img_.cols || pos.y + temp_64f_.rows > img_.rows) {
        std::cout << " The bbox is outside of the image, tracking terminantes" << '\n';
        std::exit(0);
    }
    bbox_ = BoundingBox(pos.x, pos.y, temp_64f_.cols, temp_64f_.rows);
}

void TrackerDataBase::set_template(cv::Mat temp) {
    //! set template
    temp.convertTo(temp_64f_, CV_64F);
    bbox_ = BoundingBox(0, 0, temp.rows, temp.cols);
}

cv::Point2f TrackerDataBase::get_pos() {
    return this->bbox_.center();
}

cv::Mat get_gaussian_kernel(int width, int height, double sigma) {
    cv::Point center((width - 1) / 2, (height - 1) / 2);
    cv::Mat result = cv::Mat::zeros(cv::Size(width, height), CV_64F);
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            result.at<double>(r, c) = (M_1_PI * 0.5 / (sigma * sigma)) *
                                      exp(-(pow(r - center.y, 2) + pow(c - center.x, 2)) / (2 * sigma * sigma));
        }
    }
    return result / cv::sum(result)[0];
}

void TrackerDataBase::back_up_mass_center() {
    last_bbox_ = bbox_;
}

float get_bin(float gray_value, int width_bin) {
    return gray_value / width_bin;
}

void TrackerDataBase::set_img(cv::Mat img) {
    img_ = img;
    img.convertTo(img_64f_, CV_64F);
}

std::vector<double> compute_histogram(int num_bin, cv::Mat img, cv::Mat weight) {
    assert(img.rows == weight.rows && img.cols == weight.cols);
    std::vector<double> result(num_bin, 0.0);
    int width_bin = std::ceil(255 / num_bin);
    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            int gray_value = static_cast<int>(img.at<double>(r, c));
            int bin = get_bin(gray_value, width_bin);
            // std::cout << "Gray value is: " << gray_value << ", at Bin " << bin << '\n';
            result[bin] += weight.at<double>(r, c);
        }
    }
    double sum = std::accumulate(result.begin(), result.end(), 0.0);

    std::cout << sum << '\n';
    for (double& bin_val : result) {
        bin_val /= sum;
    }
    return result;
}

cv::Mat compute_back_projection(cv::Mat img, std::vector<double> hist_temp, std::vector<double> hist_candidate) {
    assert(hist_candidate.size() == hist_temp.size());
    cv::Mat result = cv::Mat::zeros(img.size(), CV_64F);
    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            int hist_pos = get_bin(static_cast<float>(img.at<double>(r, c)), std::ceil(255 / hist_temp.size()));

            result.at<double>(r, c) = std::sqrt(hist_temp[hist_pos] / (hist_candidate[hist_pos] + 1e-8));
        }
    }
    return result;
}

std::vector<cv::Point> TrackerDataBase::get_positions() {
    std::vector<cv::Point> result;
    result.reserve(bbox_.area());

    cv::Point tl = static_cast<cv::Point>(bbox_.top_left());
    cv::Point br = static_cast<cv::Point>(bbox_.bottom_right());

    for (int y = tl.y; y < br.y; y++) {
        for (int x = tl.x; x < br.x; x++) {
            result.emplace_back(x, y);
        }
    }

    return result;
}

cv::Point2f compute_weighted_average(const std::vector<cv::Point>& data, cv::Mat weight) {
    assert(data.size() == weight.rows);

    double sum_x = 0.0, sum_y = 0.0;
    for (int i = 0; i < data.size(); i++) {
        double w = weight.at<double>(i);

        sum_x += data[i].x * w;
        sum_y += data[i].y * w;
    }

    double sum_w = cv::sum(weight)[0];

    return cv::Point2f(sum_x / sum_w, sum_y / sum_w);
}

double compute_energy(std::vector<double> hist_lhs, std::vector<double> hist_rhs) {
    assert(hist_lhs.size() == hist_rhs.size());
    double result = 0.0;
    for (int i = 0; i < hist_lhs.size(); i++) {
        result = std::sqrt(hist_lhs[i] * hist_rhs[i]);
    }

    return result;
}

cv::Mat TrackerDataBase::compute_back_projection_weight(int num_bin, double sigma) {
    cv::Mat weight = get_gaussian_kernel(this->temp_64f_.cols, this->temp_64f_.rows, sigma);
    std::vector<double> hist_temp = compute_histogram(num_bin, this->temp_64f_, weight);
    cv::Mat candidate =
        get_sub_image_from_ul(this->img_64f_, bbox_.top_left().x, bbox_.top_left().y, bbox_.width(), bbox_.height());
    std::vector<double> hist_candidate = compute_histogram(num_bin, candidate, weight);
    cv::Mat back_proj_weight = compute_back_projection(candidate, hist_temp, hist_candidate);
    // double energy = compute_energy(hist_temp, hist_candidate);
    // std::cout << "@@@@@@@@@@@ Energy is " << energy << '\n';
    return back_proj_weight;
}

cv::Point2f TrackerDataBase::compute_mean_shift(cv::Mat back_projection_weight, double sigma) {
    cv::Mat gaussian_weight = get_gaussian_kernel(temp_64f_.cols, temp_64f_.rows, sigma);
    cv::Mat weight = gaussian_weight.mul(back_projection_weight);

    std::vector<cv::Point> positions = get_positions();
    return compute_weighted_average(positions, weight.reshape(0, positions.size()));
}

void TrackerDataBase::visualize() {
    cv::Mat vis;
    cv::cvtColor(img_, vis, cv::COLOR_GRAY2BGR);
    draw_bounding_box_vis_image(vis, bbox_.top_left().x, bbox_.top_left().y, bbox_.width(), bbox_.height());

    cv::imshow("tracking result", vis);
    cv::waitKey(100);
}
