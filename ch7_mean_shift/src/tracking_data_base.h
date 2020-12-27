#pragma once
#include "bounding_box.h"
#include "data_base.h"
#include <opencv2/core/core.hpp>
#include <vector>
class TrackerDataBase : public DataBase {
   public:
    ~TrackerDataBase() = default;
    TrackerDataBase() = default;

    cv::Point2f get_object_center() const;

    void init_mass_center() override;
    void update_mass_center() override;
    bool is_convergent() override;
    void back_up_mass_center() override;
    double compute_energy() override;
    void visualize() override;
    void visualize_tracking_result();
    void set_obj_predicted_initial_center(cv::Point2f pos);
    bool iteration_call_back() override;
    void set_img(cv::Mat img);
    cv::Point2f get_pos();
    void set_template(cv::Mat temp);
    cv::Mat compute_back_projection_weight(int num_bin, double sigma);

    cv::Point2f compute_mean_shift(cv::Mat back_projection_weight, double sigma);
    cv::Mat get_candidate();
    std::vector<cv::Point2f> get_positions();

   private:
    cv::Mat img_;
    cv::Mat img_64f_;
    cv::Mat compute_kernel_weight();
    cv::Mat temp_64f_;
    BoundingBox bbox_;
    BoundingBox last_bbox_;
    bool initialized_ = false;
    std::vector<double> hist_temp_;
    std::vector<double> hist_candidate_;
    // int num_bin_;
    // double sigma_;
};