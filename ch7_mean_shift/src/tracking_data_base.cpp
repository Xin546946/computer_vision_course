#include "tracking_data_base.h"
#include "data_base.h"
#include "memory"
#include "opencv_utils.h"
#include "visualizer.h"
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>

TrackerDataBase::TrackerDataBase(cv::Mat img, cv::Mat temp, cv::Point2f initial_pos)
    : img_(img),
      temp_(temp),

      bbox_(initial_pos.x - temp_.cols / 2.0f, initial_pos.y - temp_.rows / 2.0f, static_cast<float>(temp_.cols),
            static_cast<float>(temp_.rows)) {
}

cv::Point2f TrackerDataBase::get_tracking_result() const {
    return bbox_.center();
}

void TrackerDataBase::init_mass_center() {
}

void TrackerDataBase::update_mass_center() {
    last_bbox_ = bbox_;
    int num_bin = 10;
    double sigma = 4.0;
    cv::Mat back_proj_weight = compute_back_projection_weight(num_bin, sigma);
    cv::Point2f mean_shift = compute_mean_shift(back_proj_weight, simga);
}

bool TrackerDataBase::is_convergent() {
    return cv::norm(bbox_.center() - last_bbox_.center()) < 1e-4;
}

void TrackerDataBase::set_pos(cv::Point2f pos) {
    //! update bounding box using this pose
    bbox_.move(pos.x - bbox_.center().x, pos.y - bbox_.center().y);
}

void TrackerDataBase::set_template(cv::Mat temp) {
    //! set template
    this->temp_ = temp;
}

cv::Point2f TrackerDataBase::get_pos() {
    return this->bbox_.center();
}

cv::Mat TrackerDataBase::compute_back_projection_weight(int num_bin, double sigma) {
    cv::Mat weight = compute_gaussian_kernel(this->temp_.cols, this->temp_.rows, sigma);
    cv::Mat hist_temp = compute_histogram(num_bin, this->temp_);
    cv::Mat candidate = get_candidate();
    cv::Mat hist_candidate = compute_histogram(num_bin, candidate);
    cv::Mat back_proj_weight = compute_back_projection(this->img_, hist_temp, hist_candidate);
}

cv::Mat compute_gaussian_kernel(int width, int height, double sigma) {
    cv::Mat result = cv::Mat::zeros(cv::Size(width, height), CV_64F);
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            result.at<double>(r, c) =
                (M_1_PI * 0.5 / (sigma * sigma)) * exp(-(pow(r, 2) + pow(c, 2)) / (2 * sigma * sigma));
        }
    }
    return result / cv::sum(result)[0];
}

int get_bin(int gray_value, int width_bin) {
    return gray_value / width_bin;
}

void TrackerDataBase::set_img(cv::Mat img) {
    img_ = img;
}

cv::Mat compute_histogram(int num_bin, cv::Mat img) {
}

cv::Mat compute_back_projection(cv::Mat img, cv::Mat hist_temp, cv::Mat hist_candidate) {
}

std::vector<cv::Point> TrackerDataBase::get_positions() {
}

template <typename T, typename T2>
T compute_weighted_average(std::vector<T> data, T2 weight) {
}

cv::Point2f TrackerDataBase::compute_mean_shift(cv::Mat back_projection_weight, double sigma) {
    cv::Mat gaussian_weight = compute_gaussian_kernel(temp_.cols, temp_.rows, sigma);
    cv::Mat weight = gaussian_weight.mul(back_projection_weight);
    std::vector<cv::Point> positions = get_positions();
    return compute_weighted_average(positions, weight);
}