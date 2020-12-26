#pragma once
#include "data_base.h"
#include <opencv2/core/core.hpp>

class TrackerDataBase : public DataBase {
   public:
    ~TrackerDataBase() = default;
    TrackerDataBase(cv::Mat img, cv::Mat temp, cv::Point2f initial_pos);

    void init_mass_center() override{};
    void update_mass_center() override;
    bool is_convergent() override;
    void back_up_mass_center() override;

    void visualize() override;
    void set_initial_pos(cv::Point2f initial_pos);

   private:
    cv::Rect2f bbox_;
    cv::Mat img_;
    cv::Mat temp_;
    cv::Point2f initial_pos_;
};