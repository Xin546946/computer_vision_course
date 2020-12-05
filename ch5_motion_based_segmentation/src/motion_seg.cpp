#include "motion_seg.h"
#include "display.h"
#include "opencv_utils.h"
MotionSeg::MotionSeg(int rows, int cols, int num_gaussian, const gmm::ConfigParam& config) : gmm_map(rows * cols) {
    gmm::GMM::config_param_ = config;
    gmm::GMM::num_gaussians_ = num_gaussian;
}
void MotionSeg::process(const std::vector<cv::Mat>& videos) {
    assert(!videos.empty());

    const int cols = videos[0].cols;
    const int rows = videos[0].rows;

    for (int i = 0; i < videos.size(); i++) {
        cv::Mat img_64;
        videos[i].convertTo(img_64, CV_64FC1);

        cv::Mat vis_fore_seg = cv::Mat::zeros(videos[0].size(), videos[0].type());

        for (int r = 0; r < img_64.rows; r++) {
            for (int c = 0; c < img_64.cols; c++) {
                auto& gmm = gmm_map[pos_to_id(r, c, cols)];
                double sample = img_64.at<double>(r, c);
                gmm.add_sample(sample);
                if (gmm.is_in_foreground(sample)) {
                    vis_fore_seg.at<uchar>(r, c) = 255;
                }
            }
        }

        cv::hconcat(videos[i], vis_fore_seg, vis_fore_seg);

        cv::imshow("left: original video, right : segmentation ", vis_fore_seg);
        cv::waitKey(1);
    }
}