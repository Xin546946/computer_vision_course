#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

struct ParamGaussian {
    ParamGaussian() = default;
    ParamGaussian(cv::Size size, float sigma_x, float sigma_y) : size_(size), sigma_x_(sigma_x), sigma_y_(sigma_y) {
    }
    cv::Size size_ = cv::Size(3, 3);
    float sigma_x_ = 3.0;
    float sigma_y_ = 3.0;
};

struct ParamLaplacian {
    ParamLaplacian() = default;
    ParamLaplacian(int ddepth, int kernel_size, int scale, int delta)
        : ddepth_(ddepth), kernel_size_(kernel_size), scale_(scale), delta_(delta) {
    }
    int ddepth_ = CV_16S;
    int kernel_size_ = 3;
    int scale_ = 1;
    int delta_ = 0;
};

struct ParamThreshold {
    ParamThreshold() = default;
    ParamThreshold(int threshold, int max_val, int threshold_type)
        : threshold_(threshold), max_val_(max_val), threshold_type_(threshold_type) {
    }
    int threshold_ = 75;
    int max_val_ = 255;
    int threshold_type_ = 3;
};

class LoG {
   public:
    LoG(cv::Mat img, ParamGaussian param_gaussian, ParamLaplacian param_laplacian, ParamThreshold param_thres)
        : img_(img),
          result_(img.clone()),
          param_gaussian_(param_gaussian),
          param_laplacian_(param_laplacian),
          param_thres_(param_thres) {
    }
    void run();

   private:
    cv::Mat img_;
    cv::Mat result_;
    ParamGaussian param_gaussian_;
    ParamLaplacian param_laplacian_;
    ParamThreshold param_thres_;
};
