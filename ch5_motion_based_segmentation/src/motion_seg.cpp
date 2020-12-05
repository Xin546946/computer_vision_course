/**
______________________________________________________________________
*********************************************************************
* @brief  This file is developed for the course of ShenLan XueYuan:
* Fundamental implementations of Computer Vision
* all rights preserved
* @author Xin Jin, Zhaoran Wu
* @contact: xinjin1109@gmail.com, zhaoran.wu1@gmail.com
*
______________________________________________________________________
*********************************************************************
**/
#include "motion_seg.h"
#include "display.h"
#include "opencv_utils.h"
MotionSeg::MotionSeg(int rows, int cols, int num_gaussian, const gmm::ConfigParam& config) : gmm_map(rows * cols) {
    gmm::GMM::set_config(config);
    gmm::GMM::set_num_gaussian(num_gaussian);
    std::for_each(gmm_map.begin(), gmm_map.end(),
                  [=](gmm::GMM& model) { model.model_param().param_.resize(num_gaussian); });
}

void post_processing(cv::Mat img) {
    // int sz = 1;
    // cv::Mat element =
    //    cv::getStructuringElement(cv::MorphShapes::MORPH_RECT, cv::Size(2 * sz + 1, 2 * sz + 1), cv::Point(sz, sz));
    // cv::morphologyEx(img, img, cv::MorphTypes::MORPH_OPEN, element);
    cv::medianBlur(img, img, 3);
    int sz = 3;
    cv::Mat element =
        cv::getStructuringElement(cv::MorphShapes::MORPH_RECT, cv::Size(2 * sz + 1, 2 * sz + 1), cv::Point(sz, sz));
    cv::morphologyEx(img, img, cv::MorphTypes::MORPH_CLOSE, element);
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
        post_processing(vis_fore_seg);
        cv::hconcat(videos[i], vis_fore_seg, vis_fore_seg);

        cv::imshow("left: original video, right : segmentation ", vis_fore_seg);
        cv::waitKey(1);
    }
}