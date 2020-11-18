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

#pragma once
#include "gradient_descent_base.h"
#include "level_set_utils.h"
#include "sdf_map.h"
/**
 * @brief Parameter of Level Set of CV Model
 *
 */
struct ParamLevelSet {
    ParamLevelSet(double forground_weight, double background_weight, double eps,
                  double step_size, double length_term_weight,
                  double gradient_term_weight);
    double forground_weight_;
    double background_weight_;
    double eps_;  // H(z,eps)
    double step_size_;
    double length_term_weight_;
    double gradient_term_weight_;
};
/**
 * @brief class of Level Set CV Model
 *
 */
class LevelSetCV : public GradientDescentBase {
   public:
    LevelSetCV(cv::Mat image,
               const ParamLevelSet& param);  // todo give index of the const
    void initialize() override;              // todo SDFMap and cf, cb

    void update() override;
    // todo think about testing!!!
    void update_center();
    void update_level_set();

    double compute_regu_length() const;
    double compute_regu_sdf_gradient() const;

    void roll_back_state() override;
    void back_up_state() override;
    void print_terminate_info() const override;
    double compute_energy() const override;
    std::string return_drive_class_name() const;

   private:
    SDFMap level_set_;
    SDFMap last_level_set_;

    ParamLevelSet param_;  // use param in the space of Level Set, no need for
                           // naming level set anymore
    double grayvalue_background_;
    double grayvalue_forground_;
    cv::Mat image_64f_;
    cv::Mat image_3_channel;
    double center_foreground_;
    double center_background_;
    double last_center_foreground_;
    double last_center_background_;
};
