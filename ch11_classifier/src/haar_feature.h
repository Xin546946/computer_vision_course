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
#include "matrix.h"
#include "opencv_utils.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

class HaarRect;
class DetectionWindow;

/**
 * @brief compute rect integral img from ul
 *
 * @param integral_img
 * @param row
 * @param col
 * @param width
 * @param height
 * @return double
 */
double compute_rect_integral_img_from_ul(cv::Mat integral_img, int row, int col, int width, int height);

/**
 * @brief make integral img
 *
 * @param img
 * @return cv::Mat
 */
cv::Mat make_integral_img(const cv::Mat& img);

/**
 * @brief compute haar feature vector with detection window
 *
 * @param detection_window
 * @param integral_img
 * @param x
 * @param y
 * @return cv::Mat
 */
cv::Mat compute_haar_feature_vector(DetectionWindow detection_window, cv::Mat integral_img, int x, int y);

/**
 * @brief compute haar feature matrix
 *
 * @param detection_window
 * @param integral_img
 * @return Matrix<cv::Mat>
 */
Matrix<cv::Mat> compute_haar_feature_matrix(DetectionWindow detection_window, cv::Mat integral_img);

/**
 * @brief HaarSubRect is contained by HarrRect
 *
 */
struct HaarSubRect {
    HaarSubRect(char sign, cv::Point2i ul, int width, int height)
        : sign_(sign), ul_(ul), width_(width), height_(height) {
    }
    char sign_ = -1;
    cv::Point2i ul_ = cv::Point2i(-1, -1);  // relative to HaarRect ul
    int width_ = -1;
    int height_ = -1;
};

/**
 * @brief DetectionWindow has some HarrRects
 *
 */
class DetectionWindow {
   public:
    DetectionWindow(int width, int height) : width_(width), height_(height) {
    }

    /**
     * @brief add haar features
     *
     * @param x_ratio
     * @param y_ratio
     * @param width_ratio
     * @param height_ratio
     * @param indicator_matrix
     */
    void add_haar_rect(double x_ratio, double y_ratio, double width_ratio, double height_ratio,
                       cv::Mat indicator_matrix);

    /**
     * @brief Show all sub img for all haar rect window
     *
     * @param img
     * @param x
     * @param y
     */
    void show_all_sub_image(cv::Mat img, int x, int y);
    std::vector<HaarRect> get_haar_rects() {
        return haar_rects_;
    }

    int get_num_haar_rect() const {
        return haar_rects_.size();
    }

    int get_width() const {
        return width_;
    }

    int get_height() const {
        return height_;
    }

   private:
    int width_;
    int height_;
    std::vector<HaarRect> haar_rects_;
};

/**
 * @brief HaarRect has some HarrSubRect
 *
 */
class HaarRect {
   public:
    HaarRect(int x, int y, int width, int height, cv::Mat indicator_matrix);
    cv::Point ul() {
        return ul_;
    }
    int get_num_sub_rect_x() const {
        return num_sub_rect_x_;
    }
    int get_num_sub_rect_y() const {
        return num_sub_rect_y_;
    }
    int get_width_sub_rect() const {
        return width_sub_rect_;
    }

    int get_height_sub_rect() const {
        return height_sub_rect_;
    }
    std::vector<HaarSubRect> get_haar_sub_rects() {
        return haar_sub_rects_;
    }

   private:
    cv::Point ul_;
    std::vector<HaarSubRect> haar_sub_rects_;
    int num_sub_rect_x_;
    int num_sub_rect_y_;
    int width_sub_rect_;
    int height_sub_rect_;
};
