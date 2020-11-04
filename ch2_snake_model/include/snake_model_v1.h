#include <opencv2/core.hpp>
#include <vector>
struct Sample {
    cv::Mat derivative_x_;
    cv::Mat derivative_y_;
    cv::Mat laplacian_;
    cv::Mat gaussian_;
    float feature_;
    int row_;  // row in original image
    int col_;  // col in original image
};
// TODO maybe need to be adjusted
struct Contour {
    Contour(const int num_points, const cv::Point center, const int radius)
        : num_points_(num_points), center_(center), radius_(radius){};
    int num_points_;
    std::vector<cv::Point> contours_;

    cv::Point center_;
    int radius_;
};

struct External_Energy_Param {
    External_Energy_Param(const float w_line, const float w_edge,
                          const float w_term)
        : w_line_(w_line), w_edge_(w_edge), w_term_(w_term){};
    float w_line_;
    float w_edge_;
    float w_term_;
};

struct Internal_Matrix_Param {
    Internal_Matrix_Param(const float w_membra_energy,
                          const float w_thin_plate_energy)
        : w_membra_energy_(w_membra_energy),
          w_thin_plate_energy_(w_thin_plate_energy){};
    float w_membra_energy_;
    float w_thin_plate_energy_;
};

struct GVF_Param {
    GVF_Param(const float mu, const float max_iteration, float sigma)
        : mu_(mu), max_iterations_(max_iteration), sigma_(sigma){};
    float mu_;
    int max_iterations_;
    float sigma_;
};

struct Snake_Move_Param {
    Snake_Move_Param(const float time_step, const float w_external_field,
                     const float w_ballon_force, const int max_iterations)
        : time_step_(time_step),
          w_external_field_(w_external_field),
          w_ballon_force_(w_ballon_force),
          max_iterations_(max_iterations){};
    float time_step_;
    float w_external_field_;
    float w_ballon_force_;
    int max_iterations_;
};

// TODO Energy can also be abstracted as a class!! Think
// TODO how to separate gvf and snake
class Snake_Model {
   public:
    std::vector<Sample> get_result_samples() const;
    // parameter: img,
    Snake_Model(cv::Mat img, Contour contour,
                Internal_Matrix_Param internal_param,
                External_Energy_Param external_param,
                Snake_Move_Param snake_move_param);
    void run();
    std::vector<cv::Point> get_contour();

   private:
    void initial_contour();
    void calcu_external_force_image();
    void calcu_gvf();
    void calcu_ballon_force();
    void calcu_internal_force_matrix();
    void update_snake();

    Contour contour_;
    Internal_Matrix_Param internal_param_;
    External_Energy_Param external_param_;
    GVF_Param gvf_param_;
    Snake_Move_Param snake_move_param_;
    std::vector<Sample> samples_;
};