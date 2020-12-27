#pragma once
#include "gkdtree.h"
#include <memory>
#include <opencv2/core/core.hpp>
#include <queue>
#include <unordered_set>
class Visualizer;
class DataBase {
   public:
    DataBase() = default;
    virtual ~DataBase() = default;
    virtual void init_mass_center() = 0;
    virtual void update_mass_center() = 0;
    virtual bool is_convergent() = 0;
    virtual void back_up_mass_center() = 0;
    virtual double compute_energy() {
        return 0.0;
    };
    virtual bool iteration_call_back(){};

    virtual void visualize(){};
    double energy_ = 0.0;

   private:
};

class ColorData : public DataBase {
   public:
    ~ColorData() = default;
    ColorData(cv::Mat img, double radius, std::shared_ptr<Visualizer> vis_ptr);
    void init_mass_center() override{};
    void update_mass_center() override;
    bool is_convergent() override;
    void back_up_mass_center() override;

    void visualize() override;

   private:
    double r_square_;
    std::vector<cv::Vec3f> colors_;
    std::vector<cv::Vec3f> colors_last_;
    std::vector<cv::Vec3f> colors_original_;
    std::shared_ptr<Visualizer> vis_ptr_;
};

struct BGR {
    BGR() = default;
    BGR(cv::Vec3b bgr, int row, int col) : bgr_(bgr), row_(row), col_(col) {
    }
    BGR(uchar b, uchar g, uchar r, int row, int col) : bgr_(b, g, r), row_(row), col_(col) {
    }
    // interface for gkdtree;
    typedef int DistType;
    uchar operator[](int idx) const {
        return this->bgr_(idx);
    }
    static bool is_in_radius(const BGR* center, const BGR* data, float r_square) {
        return cv::norm(center->bgr_ - data->bgr_, cv::NORM_L2SQR) < r_square;
    }
    static bool is_in_radius(const BGR* center, const BGR* data, int axis, float radius) {
        return std::abs(center->bgr_(axis) - data->bgr_(axis)) < radius;
    }

    int row_ = -1;
    int col_ = -1;

    cv::Vec3f bgr_;
    bool is_convergent_ = false;
    static const int dim_ = 3;
};

typedef GKdTree<BGR> ColorKdTree;

class BGRData : public DataBase {
   public:
    ~BGRData() = default;
    BGRData(cv::Mat img, double radius, std::shared_ptr<Visualizer> vis_ptr);

    void init_mass_center() override{};
    void update_mass_center() override;
    bool is_convergent() override;
    void back_up_mass_center() override;

    void visualize() override;

   private:
    int radius_;
    float quarter_r_square_;

    std::vector<BGR> colors_;
    std::queue<BGR*> traverse_queue_;

    std::shared_ptr<Visualizer> vis_ptr_;
    ColorKdTree* kdtree_;
};