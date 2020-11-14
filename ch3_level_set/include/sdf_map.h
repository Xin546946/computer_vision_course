#pragma once
#include <opencv2/core.hpp>

class SDFMap {  // phi
   public:
    /**
     * @brief Construct a new SDFMap object, Output a SDF
     *
     * @param center
     * @param radius
     */
    SDFMap(int rows, int cols, cv::Point2d center, double radius);
    friend cv::Mat heaviside(
        const SDFMap& sdf_map,
        double eps);  // if phi>0, H(phi) = 1; if phi<0, H(phi) = 0;
    friend cv::Mat draw_sdf_map(const SDFMap& sdf_map);
    friend cv::Mat dirac(const SDFMap& sdf_map, double eps);

    // todo define a heaviside function according to the std::for_each
    // todo using overload function for heaviside function
    void update(cv::Mat step);

    // void get_length();  // get the total length of all contours

    cv::Mat get_contour_points() const;
    double get_gradient_magnitude_level_set();
    int get_num_segment() const;  // get how many contours ***Hard***
    cv::Mat get_fore_background_label_map()
        const;  // get segment result, forground background

   private:
    cv::Mat map_;
};