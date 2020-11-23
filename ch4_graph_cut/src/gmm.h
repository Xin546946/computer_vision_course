// GMM template
// template <typename TFeature, typename DFeature, typename NGauss>
// In this case, fit a 3D GMM with 2 Gaussian
#pragma once
#include "em.h"
#include <opencv2/core/core.hpp>
#include <vector>

class GMM : public EMBase {
   public:
    GMM(cv::Mat, int num_gaussian, int max_iteration);

   private:
    void initialize() override;
    void update_e_step() override;
    void update_m_step() override;
};