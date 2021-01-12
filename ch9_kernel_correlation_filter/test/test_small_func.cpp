#include "bounding_box.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv) {
    cv::Mat_<float> nom(2, 2);
    nom << 1.f, 1.f, 1.f, 1.f;
    cv::Mat_<float> den(2, 2);
    den << 2.f, 2.f, 2.f, 2.f;
    cv::Mat result;
    cv::divide(nom, den, result, -1.0);
    std::cout << result << '\n';

    return 0;
}