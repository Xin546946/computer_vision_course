#include "histogram.h"
#include "bounding_box.h"
#include "opencv_utils.h"
/**
 * @brief make a histogramm of a given image, the resolution is control with num_bins.
 *
 * @param [in] img
 * @param [in] num_bins
 * @param [in] weight : how large is the element contribute to the histo gramm
 * @return Histogram
 */
Histogram make_histogramm(cv::Mat img, int num_bins, cv::Mat weight) {
    Histogram hist(num_bins, 0.0, 255.0);

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
Histogram make_histogramm(cv::Mat img, const BoundingBox& bbox, int num_bins, cv::Mat weight) {
    cv::Point2f up_left = bbox.top_left();
    cv::Mat sub_img = get_sub_image_from_ul(img, up_left.x, up_left.y, bbox.width(), bbox.height());

    assert(sub_img.cols == weight.cols && sub_img.rows == weight.rows);

    return make_histogramm(sub_img, num_bins, weight);
}