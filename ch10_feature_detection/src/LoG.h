/**
______________________________________________________________________
*********************************************************************
* @brief This file is developed for the course of ShenLan XueYuan:
* Fundamental implementations of Computer Vision
* all rights preserved
* @author Xin Jin, Zhaoran Wu
* @contact: xinjin1109@gmail.com, zhaoran.wu1@gmail.com
*
______________________________________________________________________
*********************************************************************
**/
#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
/**
 * @brief struct of gaussian parameter
 *
 */
struct ParamGaussian {
    /**
     * @brief Construct a new Param Gaussian object by default
     *
     */
    ParamGaussian() = default;
    /**
     * @brief Construct a new Param Gaussian object
     *
     * @param size
     * @param sigma_x
     * @param sigma_y
     */
    ParamGaussian(cv::Size size, float sigma_x, float sigma_y) : size_(size), sigma_x_(sigma_x), sigma_y_(sigma_y) {
    }
    cv::Size size_ = cv::Size(3, 3);
    float sigma_x_ = 3.0;
    float sigma_y_ = 3.0;
};

/**
 * @brief struct of parameter laplacian
 *
 */
struct ParamLaplacian {
    /**
     * @brief Construct a new Param Laplacian object by defalut
     *
     */
    ParamLaplacian() = default;
    /**
     * @brief Construct a new Param Laplacian object
     *
     * @param ddepth
     * @param kernel_size
     * @param scale
     * @param delta
     */
    ParamLaplacian(int ddepth, int kernel_size, int scale, int delta)
        : ddepth_(ddepth), kernel_size_(kernel_size), scale_(scale), delta_(delta) {
    }
    int ddepth_ = CV_16S;
    int kernel_size_ = 3;
    int scale_ = 1;
    int delta_ = 0;
};

/**
 * @brief struct of paramter os threshold
 *
 */
struct ParamThreshold {
    /**
     * @brief Construct a new Param Threshold object by default
     *
     */
    ParamThreshold() = default;
    /**
     * @brief Construct a new Param Threshold object
     *
     * @param threshold
     * @param max_val
     * @param threshold_type
     */
    ParamThreshold(int threshold, int max_val, int threshold_type)
        : threshold_(threshold), max_val_(max_val), threshold_type_(threshold_type) {
    }
    int threshold_ = 80;
    int max_val_ = 255;
    int threshold_type_ = cv::THRESH_BINARY;
};

/**
 * @brief class Laplacian of Gaussian
 *
 */
class LoG {
   public:
    /**
     * @brief Construct a new Lo G object
     *
     * @param img
     * @param param_gaussian
     * @param param_laplacian
     * @param param_thres
     */
    LoG(cv::Mat img, ParamGaussian param_gaussian, ParamLaplacian param_laplacian, ParamThreshold param_thres)
        : img_(img),
          result_(img.clone()),
          param_gaussian_(param_gaussian),
          param_laplacian_(param_laplacian),
          param_thres_(param_thres) {
    }
    /**
     * @brief process of LoG
     *
     */
    void run();

   private:
    cv::Mat img_;
    cv::Mat result_;
    ParamGaussian param_gaussian_;
    ParamLaplacian param_laplacian_;
    ParamThreshold param_thres_;
};
