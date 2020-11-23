#include "display.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_COLOR);
    GMM gmm(img, 2, 20);  // input: image, num_model, num_iteration
    gmm.run();
    gmm.get_model_param();  // get miu,sigma, w
    // test each point w.r.t. the GMM
    cv::Mat prob = cv::Mat::zeros(img.size(), CV_64F);
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            prob.at<double>(r, c) = gmm.get_prob(r, c);
        }
    }
    disp_image(prob, "clustering result w.r.t. the total model", 0);

    // test each point w.r.t. the first model
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            prob.at<double>(r, c) = gmm.get_sub_prob(r, c, 1);
        }
    }
    disp_image(prob, "clustering result w.r.t. the total model", 0);

    // test each point w.r.t. the second model
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            prob.at<double>(r, c) = gmm.get_sub_prob(r, c, 0);
        }
    }
    disp_image(prob, "clustering result w.r.t. the total model", 0);

    return 0;
}
