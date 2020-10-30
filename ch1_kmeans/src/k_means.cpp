#include "k_means.h"
#include <algorithm>
#include <vector>

// to generate random number
static std::random_device rd;
static std::mt19937 rng(rd());

/**
 * @brief get_random_index, check_convergence, calc_square_distance are helper
 * functions, you can use it to finish your homework:)
 *
 */

std::set<int> get_random_index(int max_idx, int n);

float check_convergence(const std::vector<Center>& current_centers,
                        const std::vector<Center>& last_centers);

inline float calc_square_distance(const std::array<float, 3>& arr1,
                                  const std::array<float, 3>& arr2);

/**
 * @brief Construct a new Kmeans object
 *
 * @param img : image with 3 channels
 * @param k : wanted number of cluster
 */
Kmeans::Kmeans(cv::Mat img, const int k) {
    centers_.resize(k);
    last_centers_.resize(k);
    samples_.reserve(img.rows * img.cols);

    // save each feature vector into samples
    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            std::array<float, 3> tmp_feature;
            for (int channel = 0; channel < 3; channel++) {
                tmp_feature[channel] =
                    static_cast<float>(img.at<cv::Vec3b>(r, c)[channel]);
            }
            samples_.emplace_back(tmp_feature, r, c, -1);
        }
    }
}

/**
 * @brief move the centers according to new lables
 *
 */
void Kmeans::update_centers() {
    // backup centers of last iteration
    last_centers_ = centers_;

    std::vector<std::array<float, 3>> sum_features_per_center(
        centers_.size(), {0.0f, 0.0f, 0.0f});
    std::vector<int> num_features_per_center(centers_.size(), 0);

    // calculate the mean value of feature vectors in each cluster
    // TODO complete update centers functions.
}

/**
 * @brief change the label of each sample to the nearst center
 *
 */
void Kmeans::update_labels() {
    for (Sample& sample : samples_) {
        float min_square_dist = std::numeric_limits<float>::max();
        for (int i_label = 0; i_label < centers_.size(); i_label++) {
            float square_dist = calc_square_distance(
                sample.feature_, centers_[i_label].position_);
            if (square_dist < min_square_dist) {
                // TODO update labels of each feature 2 lines
            }
        }
    }
}

std::vector<Sample> Kmeans::get_result_samples() const {
    return samples_;
}
std::vector<Center> Kmeans::get_result_centers() const {
    return centers_;
}
/**
 * @brief Execute k means algorithm
 *                1. initialize k centers randomly
 *                2. assign each feature to the corresponding centers
 *                3. calculate new centers
 *                4. check terminate condition, if it is not fulfilled, return
 * to step 2
 * @param max_iteration
 * @param smallest_convergence_radius
 */
void Kmeans::run(int max_iteration, float smallest_convergence_radius) {
    initial_centers();
    int current_iter = 0;
    while (!is_terminate(current_iter, max_iteration,
                         smallest_convergence_radius)) {
        current_iter++;
        update_labels();
        update_centers();
    }
}
/**
 * @brief initialize k centers randomly, using set to ensure there are no
 * repeated elements
 *
 */
void Kmeans::initial_centers() {
    // TODO Write a function to initialize the centers
    // helper funtion:
}
/**
 * @brief check terminate conditions, namely maximal iteration is reached or it
 * convergents
 *
 * @param current_iter
 * @param max_iteration
 * @param smallest_convergence_rate
 * @return true
 * @return false
 */
bool Kmeans::is_terminate(int current_iter, int max_iteration,
                          float smallest_convergence_rate) const {
    // TODO Write a terminate function.
    // helper funtion: check_convergence(const std::vector<Center>&
    // current_centers, const std::vector<Center>& last_centers)

    return true;
}
/**
 * @brief Get n random numbers from 1 to parameter max_idx
 *
 * @param max_idx
 * @param n
 * @return std::set<int> A set of random numbers, which has n elements
 */
std::set<int> get_random_index(int max_idx, int n) {
    std::uniform_int_distribution<int> dist(1, max_idx + 1);

    std::set<int> random_idx;
    while (random_idx.size() < n) {
        random_idx.insert(dist(rng) - 1);
    }
    return random_idx;
}
/**
 * @brief Calculate the L2 norm of current centers and last centers
 *
 * @param current_centers current assigned centers with 3 channels
 * @param last_centers  last assigned centers with 3 channels
 * @return float
 */
float check_convergence(const std::vector<Center>& current_centers,
                        const std::vector<Center>& last_centers) {
    float convergence_rate = 0;
    for (int i_center = 0; i_center < current_centers.size(); i_center++) {
        convergence_rate +=
            calc_square_distance(current_centers[i_center].position_,
                                 last_centers[i_center].position_);
    }
    return convergence_rate;
}

/**
 * @brief calculate L2 norm of two arrays
 *
 * @param arr1
 * @param arr2
 * @return float
 */
inline float calc_square_distance(const std::array<float, 3>& arr1,
                                  const std::array<float, 3>& arr2) {
    return std::pow((arr1[0] - arr2[0]), 2) + std::pow((arr1[1] - arr2[1]), 2) +
           std::pow((arr1[2] - arr2[2]), 2);
}