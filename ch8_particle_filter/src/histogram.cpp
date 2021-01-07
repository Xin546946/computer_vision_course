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
#include "histogram.h"
#include "bounding_box.h"
#include "opencv_utils.h"
#include <numeric>
void Histogram::equalize() {
    double sum = std::accumulate(hist_.begin(), hist_.end(), 0.0);
    std::for_each(hist_.begin(), hist_.end(), [=](double& height) { height /= sum; });
}

/**
 * @brief make a histogramm of a given image, the resolution is control with num_bins.
 *
 * @param [in] img
 * @param [in] num_bins
 * @param [in] weight : how large is the element contribute to the histo gramm
 * @return Histogram
 */
Histogram make_histogramm(cv::Mat img, int num_bins, cv::Mat weight, double min, double max) {
    Histogram hist(num_bins, min, max);

    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            hist.add_data(img.at<double>(r, c), weight.at<double>(r, c));
        }
    }

    return hist;
}

/**
 * @brief make a histogramm of a sub image, the sub image is specified with an image and a bbox
 *
 * @param [in] img
 * @param [in] num_bins
 * @param [in] weight
 * @return Histogram
 */
Histogram make_histogramm(cv::Mat img, const BoundingBox& bbox, int num_bins, cv::Mat weight, double min, double max) {
    cv::Point2f up_left = bbox.top_left();
    cv::Mat sub_img = get_sub_image_from_ul(img, up_left.x, up_left.y, bbox.width(), bbox.height());

    assert(sub_img.cols == weight.cols && sub_img.rows == weight.rows);

    return make_histogramm(sub_img, num_bins, weight, min, max);
}