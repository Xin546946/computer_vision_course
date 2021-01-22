#pragma once
#include "opencv_utils.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

class HaarRect;

struct HaarSubRect {
    HaarSubRect(char sign, cv::Point2i ul, int width, int height) : sign_(sign), ul_(ul) {
    }
    char sign_ = -1;
    cv::Point2i ul_ = cv::Point2i(-1, -1);  // relative to HaarRect ul
};

class DetectionWindow {
   public:
    DetectionWindow(int width, int height) : width_(width), height_(height) {
    }
    /**
     * @brief x and y are relative to the upper left of detection window
     *
     * @param x
     * @param y
     * @param width
     * @param height
     */
    void add_haar_rect(double x_ratio, double y_ratio, double width_ratio, double height_ratio,
                       cv::Mat indicator_matrix);

    void show_all_sub_image(cv::Mat img, int x, int y);

   private:
    int width_;
    int height_;
    std::vector<HaarRect> haar_rects_;
};

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

   private:
    cv::Point ul_;
    std::vector<HaarSubRect> haar_sub_rects_;
    int num_sub_rect_x_;
    int num_sub_rect_y_;
    int width_sub_rect_;
    int height_sub_rect_;
};
