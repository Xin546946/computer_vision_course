#pragma once
#include "gradient_descent_base.h"
#include "level_set_helper_function.h"
#include "sdf_map.h"
/**
 * @brief Parameter of Level Set of CV Model
 *
 */
struct ParamLevelSetCV {
    ParamLevelSetCV(double forground_weight, double background);
    double forground_weight_;
    double background_weight_;
};
/**
 * @brief class of Level Set CV Model
 *
 */
class LevelSetCV : public GradientDescentBase {
   public:
    LevelSetCV();                // todo give index of the constructor
    void initialize() override;  // todo SDFMap and cf, cb
    void update() override;
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
};