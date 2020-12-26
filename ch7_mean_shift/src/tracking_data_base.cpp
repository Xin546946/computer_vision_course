#include "tracking_data_base.h"
#include "data_base.h"
#include "memory"
#include "opencv_utils.h"
#include "visualizer.h"
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>

cv::Mat compute_back_projection(cv::Mat img, cv::Mat hist_temp, cv::Mat hist_candidate);
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
    //! todo make sure the bbox is inside the image
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
    assert(data.size() == weight.cols);

    double sum_x = 0.0, sum_y = 0.0;
    for (int i = 0; i < data.size(); i++) {
        double w = weight.at<double>(i);

        sum_x += data[i].x * w;
        sum_y += data[i].y * w;
    }

    return cv::Point2f(sum_x / data.size(), sum_y / data.size());
}

cv::Point2f TrackerDataBase::compute_mean_shift(cv::Mat back_projection_weight, double sigma) {
    cv::Mat gaussian_weight = compute_gaussian_kernel(temp_.cols, temp_.rows, sigma);
    cv::Mat weight = gaussian_weight.mul(back_projection_weight);
    std::vector<cv::Point> positions = get_positions();
    return compute_weighted_average(positions, weight.reshape(0, positions.size()));
}