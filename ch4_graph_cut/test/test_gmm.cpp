#include "display.h"
#include "gmm.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_COLOR);

    GMM gmm(img, 2);  // input: image, num_model
    gmm.run(1);

    // // test each point w.r.t. the GMM
    cv::Mat prob1 = gmm.get_sub_prob(img, 0);
    cv::Mat prob_vis_1 = get_float_mat_vis_img(prob1);
    // cv::cvtColor(prob_vis_1, prob_vis_1, CV_GRAY2RGB);
    cv::Mat prob2 = gmm.get_sub_prob(img, 1);
    cv::Mat prob_vis_2 = get_float_mat_vis_img(prob2);
    // cv::cvtColor(prob_vis_2, prob_vis_2, CV_GRAY2RGB);
    cv::Mat vis;
    // cv::vconcat(img, prob_vis_1, vis);
    cv::vconcat(prob_vis_1, prob_vis_2, vis);
    disp_image(vis, "clustering resut", 0);

    // // test each point w.r.t. the first model
    // cv::Mat sub_prob1 = get_sub_prob(img, 0);
    // disp_image(sub_prob1, "clustering result w.r.t. the total model", 0);

    // // test each point w.r.t. the second model
    // cv::Mat sub_prob_2 = get_sub_prob(img, 1);
    // disp_image(sub_prob_2, "clustering result w.r.t. the total model", 0);

    return 0;
}
