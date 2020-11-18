#pragma once

#include "gradient_descent_base.h"
#include "height_map.h"
#include "level_set_utils.h"
/**
 * @brief Parameter of Level Set of CV Model
 *
 */
struct ParamLevelSetMP {
    ParamLevelSetMP(double weight_1, double weight_2, double weight_3,
                    double weight_4, double eps, double step_size,
                    double length_term_weight, double gradient_term_weight);
    double weight_1_;
    double weight_2_;
    double weight_3_;
    double weight_4_;
    double eps_;  // H(z,eps)
    double step_size_;
    double length_term_weight_;
    double gradient_term_weight_;
};
/**
 * @brief class of Level Set CV Model
 *
 */
class LevelSetMP : public GradientDescentBase {
   public:
    LevelSetMP(cv::Mat image,
               const ParamLevelSetMP& param);  // todo give index of the const
    void initialize() override;                // todo SDFMap and cf, cb

    void update() override;
    // todo think about testing!!!
    void update_center();
    void update_level_set_mp();

    double compute_regu_length() const;
    double compute_regu_sdf_gradient() const;

    void roll_back_state() override;
    void back_up_state() override;
    void print_terminate_info() const override;
    double compute_energy() const override;
    std::string return_drive_class_name() const;

   private:
    HeightMap level_set_1_;
    HeightMap level_set_2_;
    HeightMap last_level_set_1_;
    HeightMap last_level_set_2_;
    ParamLevelSetMP param_;  // use param in the space of Level Set, no need for
                             // naming level set anymore

    cv::Mat image_64f_;
    cv::Mat image_3_channel;
    double center_1_;
    double center_2_;

    double center_3_;
    double center_4_;
    double last_center_1_;
    double last_center_2_;
    double last_center_3_;
    double last_center_4_;
};
