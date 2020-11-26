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
    Gaussian3D() = default;
    Gaussian3D(const cv::Matx31d& miu, const cv::Matx33d& sigma);

    double compute_gaussian_pdf(const cv::Matx31d& data);
    cv::Mat computer_gaussian_pdf_map(cv::Mat img);
    cv::Matx31d get_miu() const;
    cv::Matx33d get_sigma() const;
    void set_miu(const cv::Matx31d& miu);
    void set_sigma(const cv::Matx33d& sigma);

   private:
    cv::Matx31d miu_;
    cv::Matx33d sigma_;
};
class GMM : public EMBase {
   public:
    GMM(cv::Mat img, int num_gaussian_model);
    GMM(cv::Mat img, const std::vector<cv::Point>& scribble,
        int num_gaussian_model);

    // cv::Mat get_prob(cv::Mat img);
    cv::Mat get_posterior(int id_model);
    cv::Mat get_prob();

    void update_miu(int id_model, double Nk);
    void update_sigma(int id_model, double Nk);
    void update_weight(int id_model, double Nk);

    cv::Mat img_;      // img is original image, whose size is m*n
    cv::Mat samples_;  // samples is a long vector, whose size is mn*1

    std::vector<double> w_gaussian_model_;
    std::vector<Gaussian3D> gaussian3d_model_;
    std::vector<cv::Mat> posterior_;
};