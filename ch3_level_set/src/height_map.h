#pragma once
#include <opencv2/core.hpp>

class HightMap {  // phi
   public:
    /**
     * @brief Construct a new HeightMap object : sign distance function with a
     * circular zero level set
     *
     * @param rows
     * @param cols
     * @param center
     * @param radius
     */
    HightMap(int rows, int cols, cv::Point center, double radius);
    /**
     * @brief Construct a new HaightMap object : sign distance function with a
     * rectangular zero level set
     *
     * @param rows
     * @param cols
     */
    HightMap(int rows, int cols);
    cv::Mat get_map() const;  // get map_

    void add(cv::Mat step);  // update map_
    cv::Mat get_contour_points() const;
    double get_gradient_magnitude_level_set();  // get |grad(phi)|
    cv::Mat get_fore_background_label_map()
        const;  // get segment result, forground background

   private:
    cv::Mat map_;  // cv_64F
};