#include "gmm.h"
#include <opencv2/core/core.hpp>
#include <random>
#include <set>

//! distribution need to change a interface, vector to cv::Mat
static std::random_device rd;
static std::mt19937 rng(rd());

std::set<int> get_random_index(int max_idx, int n);

/*--------------------------------------------------------
#####################implementation: Gaussian3D #####################
---------------------------------------------------------*/
Gaussian3D::Gaussian3D(const cv::Matx31d& miu, const cv::Matx33d& sigma)
    : miu_(miu), sigma_(sigma) {
}

double Gaussian3D::compute_gaussian_data(const cv::Matx31d& data) {
    double coeff = cv::pow(M_1_PI * 0.5, 3 / 2) / cv::determinant(sigma_);
    cv::Mat tmp;
    cv::exp(-0.5 * (data - miu_).t() * sigma_.inv() * (data - miu_), tmp);
    return coeff * tmp.at<double>(0, 0);
}

cv::Mat Gaussian3D::compute_gaussian_map(cv::Mat img) {
    cv::Mat result = cv::Mat::zeros(cv::Size(img.cols, img.rows), CV_64F);
    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; r < img.cols; c++) {
            result.at<double>(r, c) =
                compute_gaussian_data(img.at<cv::Vec3b>(r, c));
        }
    }
    return result;
}
cv::Matx31d Gaussian3D::get_miu() const {
    return miu_;
}

cv::Matx33d Gaussian3D::get_sigma() const {
    return sigma_;
}
/*--------------------------------------------------------
#####################implementation: GMM #####################
---------------------------------------------------------*/
GMM::GMM(cv::Mat img, int num_gaussian)
    : EMBase(),
      img_(img),
      num_gaussian_(num_gaussian),
      w_gaussian_model_(num_gaussian, 1.0 / num_gaussian),
      miu_(num_gaussian, (1, 1, 1)),
      sigma_(num_gaussian, cv::Mat::eye(cv::Size(3, 3), img.type())),
      posterior_(num_gaussian, cv::Mat::zeros(cv::Size(3, 3), img.type())) {
    std::set<int> random_idx =
        get_random_index(img.rows * img.cols - 1, num_gaussian);
    int i_gaussian = 0;
    for (auto index : random_idx) {
        for (int channel = 0; channel < img.channels(); channel++) {
            miu_[i_gaussian][channel] =
                img.at<cv::Vec3b>(index % img.rows, index / img.rows)[channel];
            i_gaussian++;
        }
    }
}

std::set<int> get_random_index(int max_idx, int n) {
    std::uniform_int_distribution<int> dist(1, max_idx + 1);
    std::set<int> random_idx;
    while (random_idx.size() < n) {
        random_idx.insert(dist(rng) - 1);
    }
    return random_idx;
}

void GMM::initialize() {
    // initialize w_gaussian_model_, miu_, sigma_
    // already initialized in ctor
}

void GMM::update_e_step() {
    cv::Mat sum = cv::Mat::zeros(img_.size(), img_.type());
    for (int i = 0; i < num_gaussian_; i++) {
        posterior_[i] = gaussian_map(img_, miu_[i], sigma_[i]);
        sum += w_gaussian_model_[i] * posterior_[i];
    }

    for (int i = 0; i < num_gaussian_; i++) {
        cv::divide(posterior_[i], sum, posterior_[i]);
    }
}

double gaussian_map(cv::Mat img, cv::Vec3f miu, cv::Mat3d sigma) {
    double det = cv::determinant(sigma);
    cv::pow()
}