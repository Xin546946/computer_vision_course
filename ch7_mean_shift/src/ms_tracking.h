/**
______________________________________________________________________
*********************************************************************
* @brief  This file is developed for the course of ShenLan XueYuan:
* Fundamental implementations of Computer Vision
* all rights preserved
* @author Xin Jin, Zhaoran Wu
* @contact: xinjin1109@gmail.com, zhaoran.wu1@gmail.com
*
______________________________________________________________________
*********************************************************************
**/
#include "bounding_box.h"
#include "histogram.h"
#include <opencv2/core.hpp>
/**
 * @brief return a gauss kernel, the center of the gauss kernal is (width/2,height/2)
 *
 * @param [in] width
 * @param [in] height
 * @param [in] sigma
 * @return cv::Mat
 */
cv::Mat compute_gauss_kernel(int width, int height, double sigma);
/**
 * @brief make a histogramm of a given image, the resolution is control with num_bins.
 *
 * @param [in] img
 * @param [in] num_bins
 * @param [in] weight : how large is the element contribute to the histo gramm
 * @return Histogram
 */
Histogram make_histogramm(cv::Mat img, int num_bins, cv::Mat weight);
/**
 * @brief mean shift tracking implementation
 *
 */
class MeanShiftTracking {
   public:
    /**
     * @brief Construct a new Mean Shift Tracking object
     *
     * @param [in] temp template image
     * @param [in] num_bin_histogramm : the number of bins of the histogramm
     */
    MeanShiftTracking(cv::Mat temp, int num_bin_histogramm = 16)
        : hist_temp_(num_bin_histogramm, 0, 255), bbox_(0, 0, temp.cols, temp.rows) {
        temp.convertTo(temp_64f_, CV_64FC1);
        weight_geometirc_ = compute_gauss_kernel(temp.cols, temp.rows, 10);

        hist_temp_ = make_histogramm(temp_64f_, num_bin_histogramm, weight_geometirc_);
    }

    /**
     * @brief tracking object given a start position on current image
     *
     * @param [in] max_iteration
     * @param [in] init_object_center
     * @param [in] img_curr
     */
    void run(int max_iteration, cv::Point2f init_object_center, cv::Mat img_curr);

    cv::Point2f get_tracked_object_center() const {
        return bbox_.center();
    }

    void visualize(cv::Mat curr_img) const;

   private:
    /**
     * @brief downweight the stepsize if the matching score not increase
     *
     * @param [in] bbox_before_update
     * @param [in] img
     * @param [in] score_before_update
     * @return true
     * @return false
     */
    bool make_sure_score_increase(const BoundingBox& bbox_before_update, cv::Mat img, double score_before_update);

    cv::Mat temp_64f_;          // template in the data type : CV_64F
    Histogram hist_temp_;       // histogramm of the template image
    cv::Mat weight_geometirc_;  // geometirc weight, which is a gaussian kernel

    BoundingBox bbox_;  // bounding box of the object to tracking
};