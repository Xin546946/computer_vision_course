// GMM template
// template <typename TFeature, typename DFeature, typename NGauss>
// In this case, fit a 3D GMM with 2 Gaussian
#pragma once
#include "em.h"
#include <opencv2/core/core.hpp>
#include <tuple>
#include <vector>

class Gaussian3D {
   public:
    //! change Mat3d to Mat due to inv and matrix operations
    Gaussian3D() = default;
    Gaussian3D(const cv::Matx31d& miu, const cv::Matx33d& sigma);

    double compute_gaussian_data(const cv::Matx31d& data);
    cv::Mat compute_gaussian_map(cv::Mat img);
    cv::Matx31d get_miu() const;
    cv::Matx33d get_sigma() const;

   private:
    cv::Matx31d miu_;
    cv::Matx33d sigma_;
};
class GMM : public EMBase {
   public:
    GMM(cv::Mat img, int num_gaussian_model);

    cv::Mat get_prob(cv::Mat img);
    cv::Mat get_sub_prob(cv::Mat img, int flag);

   private:
    void initialize() override;
    void update_e_step() override;
    void update_m_step() override;

    void update_miu();
    void update_sigma();
    void update_weight();

    cv::Mat img_;

    std::vector<double> w_gaussian_model_;

    std::vector<Gaussian3D> gaussian3d_model_;

    std::vector<cv::Mat> posterior_;
};