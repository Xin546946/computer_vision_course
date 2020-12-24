#include "gkdtree.h"
#include "tictoc.h"
#include <array>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
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
        return cv::norm(center->bgr_ - data->bgr_, cv::NORM_L2SQR) < r_square;
    }
    static bool is_in_radius(const BGR* center, const BGR* data, int axis, int radius) {
        return std::abs(center->bgr_(axis) - data->bgr_(axis)) < radius;
    }
    bool is_convergent();
    cv::Vec3f bgr_;
    static const int dim_ = 3;
};

int main(int argc, char** argv) {
    std::string file_name = "/home/kit/computer_vision_course/ch7_mean_shift/img/test_data/diesel.jpeg";
    cv::Mat img = cv::imread(file_name, cv::IMREAD_COLOR);
    // cv::resize(img, img, cv::Size(100, 100));
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
    BGR search_data(img.at<cv::Vec3b>(img.rows / 2, img.cols / 2));
    std::cout << "Starting search data from the middle \n";
    tictoc::tic();
    std::vector<BGR*> result = gkdtree.rnn_search(&search_data, 1);
    std::cout << "@@@@@@@@@RNN Search costs " << tictoc::toc() / 1e3 << "miliseconds" << '\n';
    std::vector<BGR> result_bf;

    tictoc::tic();
    for (auto d : bgr) {
        if (cv::norm(d.bgr_ - search_data.bgr_, CV_L2) < 1) {
            result_bf.emplace_back(d);
        }
    }
    std::cout << "@@@@@@@@@Brute force costs " << tictoc::toc() / 1e3 << "miliseconds" << '\n';

    return 0;
}