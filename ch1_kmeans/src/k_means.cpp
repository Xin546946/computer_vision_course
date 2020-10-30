#include "k_means.h"
#include <algorithm>
#include <set>
#include <vector>

std::set<int> get_random_index(int max_idx, int n);

float check_convergence(std::vector<Center> current_centers,
                        std::vector<Center> last_centers);

float calc_square_dist(const std::array<float, 3>& arr1,
                       const std::array<float, 3>& arr2) {
    return arr1[0] * arr2[0] + arr1[1] * arr2[1] + arr1[2] * arr2[2];
}

Kmeans::Kmeans(cv::Mat img, const int k) : rng(rd()) {
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

void Kmeans::run(int max_iteration, float smallest_convergence_rate) {
    initial_centers();
    int current_iter = 0;
    while (
        is_terminate(current_iter, max_iteration, smallest_convergence_rate)) {
        current_iter++;
        update_labels();
        update_centers();
    }
}

void Kmeans::initial_centers() {
    std::set<int> random_idx =
        get_random_index(samples_.size() - 1, centers_.size());
    int i_center = 0;

    for (auto index : random_idx) {
        centers_[i_center].position_ = samples_[index].feature_;
        i_center++;
    }
}

bool Kmeans::is_terminate(int current_iter, int max_iteration,
                          float smallest_convergence_rate) const {
    float convergence_rate = check_convergence(last_centers_, centers_);
    if (current_iter == max_iteration ||
        convergence_rate < smallest_convergence_rate)
        return true;
}

std::set<int> Kmeans::get_random_index(int max_idx, int n) const {
    std::uniform_int_distribution<int> dist(1, max_idx + 1);

    std::set<int> random_idx;
    while (random_idx.size() < n) {
        random_idx.insert(dist(rng) - 1);
    }
    return random_idx;
}

float check_convergence(std::vector<Center> current_centers,
                        std::vector<Center> last_centers) {
    float convergence_rate = 0;
    for (int i_center = 0; i_center < current_centers.size(); i_center++) {
        convergence_rate +=
            calc_square_dist(current_centers[i_center].position_,
                             last_centers[i_center].position_);
    }
    return convergence_rate;
}
