#pragma once
#include <opencv2/core/core.hpp>

class DataBase {
   public:
    virtual void init_mass_center() = 0;
    virtual void update_mass_center(const std::unique_ptr<WindowBase>& win_ptr) = 0;
    virtual void visualize() const = 0;
    virtual bool is_convergent() = 0;

   private:
};

class ColorData : public DataBase {
   public:
    void init_mass_center() override{};
    void update_mass_center(const std::unique_ptr<WindowBase>& win_ptr) override;
    void visualize() const override;
    bool is_convergent() override;

   private:
    std::vector<cv::Vec3d> colors;
};
