#pragma once
#include "gradient_descent_base.h"
#include "level_set_utils.h"
#include "sdf_map.h"
/**
 * @brief Parameter of Level Set of CV Model
 *
 */
struct ParamLevelSetCV {
    ParamLevelSetCV(double forground_weight, double background, double eps);
    double forground_weight_;
    double background_weight_;
    double eps_;  // H(z,eps)
    double step_size_;
    double length_term_weight;
    double gradient_term_weight;
};
/**
 * @brief class of Level Set CV Model
 *
 */
class LevelSetCV : public GradientDescentBase {
   public:
    LevelSetCV(
        cv::Mat image,
        const ParamLevelSetCV& param);  // todo give index of the constructor
    void initialize() override;         // todo SDFMap and cf, cb

    void update() override;
    // todo think about testing!!!
    void upadate_center();
    void update_level_set();

    double compute_regu_length() const;
    double compute_regu_sdf_gradient() const;

    void roll_back_state() override;
    void back_up_state() override;
    void print_terminate_info() const override;
    double compute_energy() const override;

   private:
    SDFMap level_set_;
    ParamLevelSetCV param_;  // use param in the space of Level Set, no need for
                             // naming level set anymore
    double grayvalue_background_;
    double grayvalue_forground_;
    cv::Mat image_;
    double center_foreground_;
    double center_background_;
};