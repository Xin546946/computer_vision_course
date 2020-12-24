#include "gkdtree.h"
#include "tictoc.h"
#include <array>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <random>
#include <string>
#include <vector>

struct BGR {
    BGR(cv::Vec3b bgr) : bgr_(bgr) {
    }
    BGR(uchar b, uchar g, uchar r) : bgr_(b, g, r) {
    }
    typedef int DistType;
    const uchar operator[](int idx) const {
        return this->bgr_(idx);
    }
    static bool is_in_radius(const BGR* center, const BGR* data, int r_square) {
        return cv::norm(center->bgr_, -data->bgr_, cv::NORM_L2SQR) < r_square;
    }
    static bool is_in_radius(const BGR* center, const BGR* data, int axis, int radius) {
        return std::abs(center->bgr_(axis) - data->bgr_(axis)) < radius;
    }
    bool is_convergent();
    cv::Vec3b bgr_;
    static const int dim_ = 3;
};

int main(int argc, char** argv) {
    std::string file_name = "/home/kit/computer_vision_course/ch7_mean_shift/img/test_data/diesel.jpeg";
    cv::Mat img = cv::imread(file_name, cv::IMREAD_COLOR);
    std::cout << "Finish reading img\n";
    std::vector<BGR> bgr;
    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            bgr.emplace_back(img.at<cv::Vec3b>(r, c));
        }
    }
    std::cout << "Finish building data type for kdtree\n";
    GKdTree<BGR> gkdtree(&bgr[0], bgr.size());
    std::cout << "Finish building KDTree\n";
    BGR search_data(100, 100, 100);
    std::cout << "Starting search data(100,100,100)\n";
    std::vector<BGR*> result = gkdtree.rnn_search(&search_data, 10);

    return 0;
}