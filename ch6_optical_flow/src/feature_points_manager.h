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
#include "bounding_box.h"
#include <opencv2/highgui/highgui.hpp>
#include <vector>
/**
 * @brief Manage feature points for optical flow tracking
 *
 */
class FeaturePointsManager {
   public:
    /**
     * @brief Construct a new Feature Points Manager object
     *
     */
    FeaturePointsManager() = default;
    /**
     * @brief initialize feature points with given images and initial bounding box
     *
     * @param img
     * @param initial_bbox
     */
    void initialize(cv::Mat img, BoundingBox initial_bbox);
    /**
     * @brief extract new feature points, when they are not enough
     *
     * @param img
     */
    void extract_new_feature_points(cv::Mat img);
    /**
     * @brief Process feature points according to the framework at slide page 25
     *
     * @param img
     * @param feature_points
     * @param status
     */
    void process_feature_points(cv::Mat img, const std::vector<cv::Point2f>& feature_points,
                                std::vector<uchar>& status);
    /**
     * @brief Get the feature points
     *
     * @return std::vector<cv::Point2f>
     */
    std::vector<cv::Point2f> get_feature_points() const {
        return feature_points_;
    };
    /**
     * @brief Get the bounding box
     *
     * @return BoundingBox
     */
    BoundingBox get_bbox() const {
        return bbox_;
    };

   private:
    /**
     * @brief Compute mask with given image size
     *
     * @param rows
     * @param cols
     * @return cv::Mat
     */
    cv::Mat compute_mask(int rows, int cols);
    /**
     * @brief update status by rigid body assumption and optical flow quality
     *
     * @param motion
     * @param status
     */
    void update_status(const std::vector<cv::Vec2f>& motion, std::vector<uchar>& status);
    /**
     * @brief
     *
     * @param [in] motion
     * @param [in] status
     * @param [in] width_img
     * @param [in] hight_img
     */
    void update_bbox(const std::vector<cv::Vec2f>& motion, std::vector<uchar>& status, int width_img, int height_img);
    /**
     * @brief update feature points with status
     *
     * @param new_feature_points
     * @param status
     */
    void update_feature_points(const std::vector<cv::Point2f>& new_feature_points, std::vector<uchar>& status);

    /**
     * @brief Mark "bad" amplitude with status 0
     *
     * @param motion
     * @param status
     * @param rate
     */
    void mark_status_with_amplitude(const std::vector<cv::Vec2f>& motion, std::vector<uchar>& status, float rate);
    /**
     * @brief Mark "bad" angle with status 0
     *
     * @param motion
     * @param status
     * @param rate
     */
    void mark_status_with_angle(const std::vector<cv::Vec2f>& motion, std::vector<uchar>& status, float rate);
    /**
     * @brief Mark feature points, which is already in the bounding box, 0
     *
     * @param status
     */
    void mark_status_with_contained_points(std::vector<uchar>& status);

    /**
     * @brief Visualize feature points
     *
     * @param img
     * @param feature_points_at_new_position
     */
    void visualize(cv::Mat img, const std::vector<cv::Point2f>& feature_points_at_new_position);
    /**
     * @brief Check if there is enough fps
     *
     * @return true
     * @return false
     */
    bool is_enough_points() const {
        return feature_points_.size() > 25;
    };

    std::vector<cv::Point2f> feature_points_;
    BoundingBox bbox_;
};
/**
 * @brief Overloaded function for +=, which returns a vec of fps including fps1 and fps2
 *
 * @param feature_points_1
 * @param feature_points_2
 * @return std::vector<cv::Point2f>&
 */
std::vector<cv::Point2f>& operator+=(const std::vector<cv::Point2f>& feature_points_1,
                                     const std::vector<cv::Point2f>& feature_points_2);