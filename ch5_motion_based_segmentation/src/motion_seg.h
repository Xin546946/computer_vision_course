#pragma once

#include "gmm.h"
#include <opencv2/core/core.hpp>

class MotionSeg {
   public:
    MotionSeg(int rows, int cols, int num_gaussian, const gmm::ConfigParam& config);
    void process(const std::vector<cv::Mat>& video);

   private:
    std::vector<gmm::GMM> gmm_map;
};