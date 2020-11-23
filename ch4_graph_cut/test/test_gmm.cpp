#include "display.h"
#include "gmm.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_COLOR);
    GMM gmm(img, 2);  // input: image, num_model, num_iteration
    gmm.run(20);
    gmm.get_param();  // get miu,sigma, w
    // test each point w.r.t. the GMM
    cv::Mat prob = gmm.get_prob(img);
    disp_image(prob, "clustering result w.r.t. the total model", 0);

    // test each point w.r.t. the first model
    cv::Mat sub_prob1 = get_sub_prob(img, 0);
    disp_image(sub_prob1, "clustering result w.r.t. the total model", 0);

    // test each point w.r.t. the second model
    cv::Mat sub_prob_2 = get_sub_prob(img, 1);
    disp_image(sub_prob_2, "clustering result w.r.t. the total model", 0);

    return 0;
}
