#include "k_means.h"

float calc_square_dist(const std::array<float, 3>& arr1,
                       const std::array<float, 3>& arr2) {
    return arr1[0] * arr2[0] + arr1[1] * arr2[1] + arr1[2] * arr2[2];
}

Kmeans::Kmeans(cv::Mat img, const int k) {
    centers_.resize(k);
    last_centers_.resize(k);

    samples_.reserve(img.rows * img.cols);

    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            std::array<float, 3> tmp_feature;
            for (int channel = 0; channel < 3; channel++) {
                tmp_feature[channel] = img.at<cv::Vec3f>(r, c)[channel];
            }
            samples_.emplace_back(tmp_feature, r, c, -1);
        }
    }
}

void Kmeans::run() {
    initial_centers();

    while (!is_terminate()) {
        update_labels();
        update_centers();
    }
}

void Kmeans::update_labels() {
    for (Sample& sample : samples_) {
        float min_square_dist = std::numeric_limits<float>::max();
        for (int i_label = 0; i_label < 3; i_label++) {
            float square_dist =
                calc_square_dist(sample.feature_, centers_[i_label].position_);
            if (square_dist < min_square_dist) {
                min_square_dist = square_dist;
                sample.label_ = i_label;
            }
        }
    }
}

void Kmeans::update_centers() {
    last_centers_ = centers_;

    std::vector<std::array<float, 3>> sum_features(centers_.size(),
                                                   {0.0f, 0.0f, 0.0f});
    std::vector<int> sum_count(centers_.size(), 0);

    for (const Sample& sample : samples_) {
        for (int channel = 0; channel < 3; channel++) {
            sum_features[sample.label_][channel] += sample.feature_[channel];
            sum_count[sample.label_]++;
        }
    }

    for (int i_center = 0; i_center < centers_.size(); i_center++) {
        for (int channel = 0; channel < 3; channel++) {
            centers_[i_center].position_[channel] =
                sum_features[i_center][channel] / sum_count[i_center];
        }
    }
}

std::vector<Sample> Kmeans::get_result() const {
    return samples_;
}
