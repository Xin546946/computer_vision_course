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
#include <vector>
/**
 * @brief struct of line param
 *
 */
struct LineParam {
    /**
     * @brief Construct a new Line Param object
     *
     */
    LineParam() = default;
    /**
     * @brief Construct a new Line Param object
     *
     * @param a
     * @param b
     */
    LineParam(double a, double b) : a_(a), b_(b) {
    }

    double a_;
    double b_;
};
/**
 * @brief Fitting a line for given points
 *
 * @param points
 * @return LineParam
 */
LineParam line_fitting(const std::vector<cv::Point2d>& points);