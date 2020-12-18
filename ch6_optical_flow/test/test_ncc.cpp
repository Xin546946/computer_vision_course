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

    return (1 / num) * cv::sum(((img1 - mean1.val[0]) / dev1.val[0]) * ((img2 - mean2.val[0]) / dev2.val[0]))[0];
}
int main(int argc, char** argv) {
    cv::Matx22f img1(1, 1, 1, 1);
    cv::Matx22f img2(2, 2, 2, 2);
    double score = compute_normalized_cross_correlation(cv::Mat(img1), cv::Mat(img2));
    std::cout << score << '\n';
    return 0;
}