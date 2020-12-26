#pragma once
#include "bounding_box.h"
#include "data_base.h"
#include <opencv2/core/core.hpp>
class TrackerDataBase : public DataBase {
   public:
    ~TrackerDataBase() = default;
    TrackerDataBase(cv::Mat img, cv::Mat temp, cv::Point2f pos);

    cv::Point2f get_tracking_result() const;

    void init_mass_center() override;
    void update_mass_center() override;
    bool is_convergent() override;
    void back_up_mass_center() override;

    void visualize() override;
    void set_pos(cv::Point2f pos);
    void set_img(cv::Mat img);
    cv::Point2f get_pos();
    void set_template(cv::Mat temp);
    cv::Mat compute_back_projection_weight(int num_bin, double sigma);
    cv::Mat compute_mean_shift(cv::Mat back_projection_weight);
    cv::Mat get_candidate();

   private:
    cv::Mat img_;
    cv::Mat compute_kernel_weight();
    cv::Mat temp_;
    cv::Mat img_;
    BoundingBox bbox_;
    BoundingBox last_bbox_;
    bool initialized_ = false;
};