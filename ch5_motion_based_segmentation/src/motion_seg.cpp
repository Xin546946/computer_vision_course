#include "motion_seg.h"
#include "display.h"
#include "opencv_utils.h"
MotionSeg::MotionSeg(int rows, int cols, int num_gaussian, const gmm::ConfigParam& config)
    : gmm_map(rows * cols, gmm::GMM(num_gaussian, config)) {
}
void MotionSeg::process(const std::vector<cv::Mat>& videos) {
    assert(!videos.empty());

    const int cols = videos[0].cols;
    const int rows = videos[0].rows;

    for (int i = 0; i < videos.size(); i++) {
        cv::Mat img_64;
        if (videos[0].type() != CV_64FC1) {
            videos[i].convertTo(img_64, CV_64FC1);
        }

        cv::Mat vis_fore_seg = cv::Mat::zeros(videos[0].size(), videos[0].type());

        for (int r = 0; r < img_64.rows; r++) {
            for (int c = 0; c < img_64.cols; c++) {
                auto& gmm = gmm_map[pos_to_id(r, c, cols)];
                double sample = videos[i].at<double>(r, c);
                gmm.add_sample(sample);
                if (gmm.is_last_sample_foreground()) {
                    vis_fore_seg.at<uchar>(r, c) = 255;
                }
            }
        }

        cv::hconcat(videos[i], vis_fore_seg, vis_fore_seg);
        disp_image(vis_fore_seg,
                   "curr frame id : " + std::to_string(i) + " left: original video, right : segmentation ", 1);
    }
}