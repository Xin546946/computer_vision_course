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
#include "ms_tracking.h"
#include "opencv_utils.h"

/**
 * @brief return a gauss kernel, the center of the gauss kernal is (width/2,height/2)
 *
 * @param [in] width
 * @param [in] height
 * @param [in] sigma
 * @return cv::Mat
 */
cv::Mat compute_gauss_kernel(int width, int height, double sigma) {
    cv::Point center((width - 1) / 2, (height - 1) / 2);
    cv::Mat result = cv::Mat::zeros(cv::Size(width, height), CV_64F);
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            result.at<double>(r, c) = (M_1_PI * 0.5 / (sigma * sigma)) *
                                      exp(-(pow(r - center.y, 2) + pow(c - center.x, 2)) / (2 * sigma * sigma));
        }
    }
    return result / cv::sum(result)[0];
}
/**
 * @brief return true if the center moving with only a small radius
 *
 * @param [in] before
 * @param [in] after
 * @return true
 * @return false
 */
inline bool is_convergent(cv::Point2f before, cv::Point2f after) {
    return cv::norm(cv::Mat(before - after), cv::NORM_L2SQR) < 0.1;
}

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
/**
 * @brief compute the matching score, given two histogramm.
 *
 * @param [in] hist_candidate
 * @param [in] hist_temp
 * @return double
 */
double compute_matching_score(const Histogram& hist_candidate, const Histogram& hist_temp) {
    assert(hist_temp.num_bin() == hist_candidate.num_bin());

    double result = 0.0;
    for (int i = 0; i < hist_temp.num_bin(); i++) {
        result += std::sqrt(hist_temp.get_bin_height(i) * hist_candidate.get_bin_height(i));
    }
    return result;
}
/**
 * @brief compute the back-projection weight given two histogramm and a sub image, the sub image is specified with an
 * image and a bbox
 *
 * @param [in] hist_candidate
 * @param [in] hist_temp
 * @param [in] img
 * @param [in] bbox
 * @return cv::Mat
 */
cv::Mat compute_back_project_weight(const Histogram& hist_candidate, const Histogram& hist_temp, cv::Mat img,
                                    BoundingBox bbox) {
    assert(hist_candidate.num_bin() == hist_temp.num_bin());
    cv::Point2f up_left = bbox.top_left();
    cv::Mat sub_img = get_sub_image_from_ul(img, up_left.x, up_left.y, bbox.width(), bbox.height());

    cv::Mat result = cv::Mat::zeros(sub_img.size(), CV_64F);

    for (int r = 0; r < result.rows; r++) {
        for (int c = 0; c < result.cols; c++) {
            int id_bin = hist_temp.get_bin_id(sub_img.at<double>(r, c));

            result.at<double>(r, c) =
                std::sqrt(hist_temp.get_bin_height(id_bin) / (hist_candidate.get_bin_height(id_bin) + 1e-8));
        }
    }

    return result;
}
/**
 * @brief update the bbox position with given weight
 *
 * @param [in] bbox
 * @param [in] weight
 * @return cv::Point2f
 */
cv::Point2f update_mass_center(BoundingBox bbox, cv::Mat weight) {
    cv::Point2f up_left = bbox.top_left();

    float sum_x = 0.0f;
    float sum_y = 0.0f;

    for (int r = 0; r < weight.rows; r++) {
        for (int c = 0; c < weight.cols; c++) {
            double w = weight.at<double>(r, c);
            sum_x += (up_left.x + c) * w;
            sum_y += (up_left.y + r) * w;
        }
    }

    float sum_w = cv::sum(weight)[0];

    return cv::Point2f(sum_x / sum_w, sum_y / sum_w);
}

/**
 * @brief downweight the stepsize if the matching score not increase
 *
 * @param [in] bbox_before_update
 * @param [in] img
 * @param [in] score_before_update
 * @return true
 * @return false
 */
bool MeanShiftTracking::make_sure_score_increase(const BoundingBox& bbox_before_update, cv::Mat img,
                                                 double score_before_update) {
    Histogram hist_curr = make_histogramm(img, bbox_, hist_temp_.num_bin(), weight_geometirc_);
    double score_after_update = compute_matching_score(hist_curr, hist_temp_);

    cv::Point2f center_before_update = bbox_before_update.center();

    int it = 0;
    int max_iteration = 10;
    while (it++ < max_iteration && score_before_update > score_after_update) {
        cv::Point2f center_curr = bbox_.center();
        cv::Point2f mid_point = calc_mid_point(center_curr, center_before_update);

        bbox_.move_center_to(mid_point.x, mid_point.y);

        hist_curr = make_histogramm(img, bbox_, hist_temp_.num_bin(), weight_geometirc_);
        float mid_point_matching_score = compute_matching_score(hist_curr, hist_temp_);

        if (std::abs(mid_point_matching_score - score_after_update) < 1e-6) {
            break;
        } else {
            score_after_update = mid_point_matching_score;
        }
    }

    if (it == max_iteration + 1) {
        bbox_ = bbox_before_update;
        return false;
    }

    return true;
}

/**
 * @brief tracking object given a start position on current image
 *
 * @param [in] max_iteration
 * @param [in] init_object_center
 * @param [in] img_curr
 */
void MeanShiftTracking::run(int max_iteration, cv::Point2f init_object_center, cv::Mat img_curr) {
    cv::Mat img_curr_64f;
    img_curr.convertTo(img_curr_64f, CV_64FC1);

    bbox_.move_center_to(init_object_center.x, init_object_center.y);
    BoundingBox bbox_before_update = bbox_;

    cv::Point2f last_object_center;

    double score_last = std::numeric_limits<double>::max();
    int it = 0;
    while (!it || (it < max_iteration && !is_convergent(last_object_center, bbox_.center()))) {
        it++;
        Histogram hist_curr = make_histogramm(img_curr_64f, bbox_, hist_temp_.num_bin(), weight_geometirc_);
        double score_before_update = compute_matching_score(hist_curr, hist_temp_);

        cv::Mat weight_back_proj = compute_back_project_weight(hist_curr, hist_temp_, img_curr_64f, bbox_);
        cv::Mat weight = weight_back_proj.mul(weight_geometirc_);

        cv::Point2f new_center = update_mass_center(bbox_, weight);
        last_object_center = bbox_.center();

        bbox_.move_center_to(new_center.x, new_center.y);

        // make sure have a better matching score
        bool score_increase = make_sure_score_increase(bbox_before_update, img_curr_64f, score_before_update);
        if (!score_increase) {
            break;
        }
    }
}

void MeanShiftTracking::visualize(cv::Mat curr_img) const {
    cv::Mat vis;
    cv::cvtColor(curr_img, vis, CV_GRAY2BGR);
    cv::Point2f up_left = bbox_.top_left();
    draw_bounding_box_vis_image(vis, up_left.x, up_left.y, bbox_.width(), bbox_.height());
    cv::imshow("Mean shift tracking", vis);
    cv::waitKey(10);
}
