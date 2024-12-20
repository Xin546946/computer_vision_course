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

const int sigma = 10;
const int num_bin = 10;

cv::Mat compute_back_projection(cv::Mat img, cv::Mat hist_temp, cv::Mat hist_candidate);

cv::Point2f TrackerDataBase::get_object_center() const {
    return bbox_.center();
}

void TrackerDataBase::init_mass_center() {
}

void TrackerDataBase::back_up_mass_center() {
    last_bbox_ = bbox_;
}

void TrackerDataBase::update_mass_center() {
    cv::Mat back_proj_weight = compute_back_projection_weight(num_bin, sigma);

    cv::Point2f mean_shift = compute_mean_shift(back_proj_weight, sigma);

    bbox_.move_center_to(mean_shift.x, mean_shift.y);
}

bool TrackerDataBase::is_convergent() {
    // return cv::norm(bbox_.center() - last_bbox_.center()) < 1e-4;
    return false;
}

void TrackerDataBase::set_obj_predicted_initial_center(cv::Point2f pos) {
    //! update bounding box using this pose
    //! todo make sure the bbox is inside the image
    if (pos.x + temp_64f_.cols > img_.cols || pos.y + temp_64f_.rows > img_.rows) {
        std::cout << " The bbox is outside of the image, tracking terminantes" << '\n';
        std::exit(0);
    }
    bbox_.move_center_to(pos.x, pos.y);
}

void TrackerDataBase::set_template(cv::Mat temp) {
    //! set template
    temp.convertTo(temp_64f_, CV_64F);
    bbox_ = BoundingBox(0, 0, temp.cols, temp.rows);
}

cv::Mat compute_gaussian_kernel(int width, int height, double sigma) {
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

std::vector<cv::Point2f> TrackerDataBase::get_positions() {
    std::vector<cv::Point2f> result;
    result.reserve(temp_64f_.rows * temp_64f_.cols);

    cv::Point2f tl = bbox_.top_left();
    cv::Point2f br = bbox_.bottom_right();

    for (float y = tl.y; y < br.y; y++) {
        for (float x = tl.x; x < br.x; x++) {
            result.emplace_back(x, y);
        }
    }

    return result;
}

cv::Point2f compute_weighted_average(const std::vector<cv::Point2f>& data, cv::Mat weight) {
    assert(data.size() == weight.rows * weight.cols);
    double sum_x = 0.0, sum_y = 0.0;

    for (int r = 0; r < weight.rows; r++) {
        for (int c = 0; c < weight.cols; c++) {
            int id = pos_to_id(r, c, weight.cols);
            double w = weight.at<double>(r, c);

            sum_x += data[id].x * w;
            sum_y += data[id].y * w;
        }
    }
    double sum_w = cv::sum(weight)[0];

    return cv::Point2f(sum_x / sum_w, sum_y / sum_w);
}

double TrackerDataBase::compute_energy() {
    cv::Mat weight = compute_gaussian_kernel(this->temp_64f_.cols, this->temp_64f_.rows, sigma);
    hist_temp_ = compute_histogram(num_bin, this->temp_64f_, weight);
    cv::Mat candidate =
        get_sub_image_from_ul(this->img_64f_, bbox_.top_left().x, bbox_.top_left().y, bbox_.width(), bbox_.height());
    hist_candidate_ = compute_histogram(num_bin, candidate, weight);

    assert(hist_temp_.size() == hist_candidate_.size());
    double result = 0.0;
    for (int i = 0; i < hist_temp_.size(); i++) {
        result += std::sqrt(hist_temp_[i] * hist_candidate_[i]);
    }
    return result;
}

bool TrackerDataBase::iteration_call_back() {
    double energy_curr = compute_energy();
    int iter = 0;
    while (energy_curr > energy_ && iter++ < 10) {
        cv::Point2f mid_point = calc_mid_point(last_bbox_.center(), bbox_.center());

        bbox_.move_center_to(mid_point.x, mid_point.y);

        energy_curr = compute_energy();
    }

    if (energy_curr > energy_) {
        bbox_ = last_bbox_;
        return true;
    }

    std::cout << "energy : " << energy_ << '\n';

    return false;
}

cv::Mat TrackerDataBase::compute_back_projection_weight(int num_bin, double sigma) {
    cv::Mat weight = compute_gaussian_kernel(this->temp_64f_.cols, this->temp_64f_.rows, sigma);
    hist_temp_ = compute_histogram(num_bin, this->temp_64f_, weight);
    cv::Mat candidate =
        get_sub_image_from_ul(this->img_64f_, bbox_.top_left().x, bbox_.top_left().y, bbox_.width(), bbox_.height());
    hist_candidate_ = compute_histogram(num_bin, candidate, weight);
    cv::Mat back_proj_weight = compute_back_projection(candidate, hist_temp_, hist_candidate_);
    // double energy = compute_energy(hist_temp, hist_candidate);
    // std::cout << "@@@@@@@@@@@ Energy is " << energy << '\n';
    return back_proj_weight;
}

cv::Point2f TrackerDataBase::compute_mean_shift(cv::Mat back_projection_weight, double sigma) {
    cv::Mat gaussian_weight = compute_gaussian_kernel(temp_64f_.cols, temp_64f_.rows, sigma);
    cv::Mat weight = gaussian_weight.mul(back_projection_weight);

    std::vector<cv::Point2f> positions = get_positions();
    std::cout << "rows: " << positions.size() << "weight size :" << weight.rows * weight.cols << '\n';
    return compute_weighted_average(positions, weight);
}

void TrackerDataBase::visualize_tracking_result() {
    cv::Mat vis;
    cv::cvtColor(img_, vis, cv::COLOR_GRAY2BGR);
    draw_bounding_box_vis_image(vis, bbox_.top_left().x, bbox_.top_left().y, bbox_.width(), bbox_.height());

    cv::imshow("tracking result", vis);
    cv::waitKey(50);
}

void TrackerDataBase::visualize() {
    cv::Mat vis;
    cv::cvtColor(img_, vis, cv::COLOR_GRAY2BGR);
    draw_bounding_box_vis_image(vis, bbox_.top_left().x, bbox_.top_left().y, bbox_.width(), bbox_.height());

    /*     cv::imshow("tracking result", vis);
        cv::waitKey(50); */
}
