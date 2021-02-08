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
#include <opencv2/core.hpp>

/**
 * @brief class Visualizer for visualizing points and lines
 *
 */
class Visualizer {
   public:
    /**
     * @brief Construct a new Visualizer objec: given is range of x,y axes
     *
     * @param x_min
     * @param x_max
     * @param y_min
     * @param y_max
     */
    Visualizer(double x_min, double x_max, double y_min, double y_max);

    /**
     * @brief add point at position (x,y), mark it as in/outlier
     *
     * @param x
     * @param y
     * @param is_inlier
     */
    void add_point(double x, double y, bool is_inlier);

    /**
     * @brief add line, user can choose to draw
     *
     * @param a
     * @param b
     * @param draw_thresh_line
     * @param threshold
     */
    void add_line(double a, double b, bool draw_thresh_line = false, double threshold = 0.0);

    /**
     * @brief display the board with lines and points
     *
     * @param delay
     */
    void show(int delay) const;

   private:
    cv::Mat board_;
    cv::Vec2i up_left_;
};