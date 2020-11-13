#include "sdf_map.h"

SDFMap::SDFMap(int rows, int cols, cv::Point2d center, double radius)
    : map_(cv::Mat::zeros(cv::Size(cols, rows), CV_64F)) {
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            map_.at<double>(r, c) =
                std::sqrt(pow(r - center.y, 2) + pow(c - center.x, 2)) - radius;
        }
    }
}
