#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

double compute_normalized_cross_correlation(cv::Mat img1, cv::Mat img2) {
    // todo test
    assert(img1.size() == img2.size());
    cv::Scalar mean1, mean2, dev1, dev2;
    cv::meanStdDev(img1, mean1, dev1);
    cv::meanStdDev(img2, mean2, dev2);
    int num = img1.rows * img1.cols;

    return (1 / (num * (dev1.val[0] + 1e-5) * (dev2.val[0] + 1e-5))) * (img1 - mean1.val[0]).dot(img2 - mean2.val[0]);
}
int main(int argc, char** argv) {
    cv::Mat img1 = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
    img1.convertTo(img1, CV_64FC1);
    img2.convertTo(img2, CV_64FC1);
    double score = compute_normalized_cross_correlation(img1, img2);
    std::cout << score << '\n';
    return 0;
}