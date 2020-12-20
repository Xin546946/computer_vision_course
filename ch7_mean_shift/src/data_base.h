#pragma once
#include <memory>
#include <opencv2/core/core.hpp>
class DataBase {
   public:
    DataBase() = default;
    virtual ~DataBase() = default;
    virtual void init_mass_center() = 0;
    virtual void update_mass_center() = 0;
    virtual void visualize() const = 0;
    virtual bool is_convergent() = 0;
    virtual void back_up_mass_center() = 0;

   private:
};

class ColorData : public DataBase {
   public:
    ColorData(cv::Mat img, double radius);
    void init_mass_center() override{};
    void update_mass_center() override;
    void visualize() const override;
    bool is_convergent() override;
    void back_up_mass_center() override;

   private:
    double r_square_;
    std::vector<cv::Vec3d> colors_;
    std::vector<cv::Vec3d> colors_back_up_;
};
