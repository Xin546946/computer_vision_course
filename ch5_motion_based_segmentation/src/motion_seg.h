#pragma once

#include "gmm.h"
#include <opencv2/core/core.hpp>

class MotionSeg {
   public:
    MotionSeg() = default;
    void process(const std::vector<cv::Mat>& video);

   private:
    std::vector<gmm::GMM> gmm_map;
};