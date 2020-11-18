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

#include "gradient_descent_base.h"
#include "level_set_cv.h"

struct ParamLevelSetLBF : public ParamLevelSet {
    ParamLevelSetLBF(double forground_weight, double background_weight,
                     double eps, double step_size, double length_term_weight,
                     double gradient_term_weight, int window_size,
                     double sigma);
    int window_size_;
    double sigma_;
};

class LevelSetLBF : public GradientDescentBase {
   public:
    LevelSetLBF(cv::Mat image, const ParamLevelSetLBF& param);
    void initialize() override;  // todo HeightMap and cf, cb

    void update() override;
    // todo think about testing!!!
    void update_center();     // todo write a sliding window in this function
    void update_level_set();  // todo write a

    double compute_regu_length() const;
    double compute_regu_height_map_gradient() const;

    void roll_back_state() override;
    void back_up_state() override;
    void print_terminate_info() const override;
    double compute_energy() const override;
    std::string return_drive_class_name() const;
    void update_center_in_window(int row, int col);
    cv::Mat compute_data_term_derivative_in_window(int row, int col) const;

   private:
    HightMap level_set_;
    HightMap last_level_set_;

    ParamLevelSetLBF param_;  // use param in the space of Level Set, no need
                              // for naming level set anymore
    double grayvalue_background_;
    double grayvalue_forground_;
    cv::Mat image_64f_;
    cv::Mat image_3_channel;
    double center_foreground_;
    double center_background_;
    double last_center_foreground_;
    double last_center_background_;

    cv::Mat gauss_kernel_;
};
