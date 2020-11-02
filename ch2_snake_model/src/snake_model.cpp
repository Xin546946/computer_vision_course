#include <opencv2/core.hpp>

struct Contour {
    int num_points;
    std::vector<cv::Point> contours;
    cv::Point center;
    int radius;
};

struct External_Energy_Param {
    float w_line_;
    float w_edge_;
    float w_term_;
};

struct Internal_Matrix_Param {
    float w_membra_energy;
    float w_thin_plate_energy;
};

struct GVF_Param {
    float mu;
    int max_iteration;
};

struct Snake_Move_Param {
    float time_step;
    float w_external_field;
    float w_ballon_force;
};

class Snake_Model {
   public:
    // parameter: img,
    Snake_Model(cv::Mat img, Contour contour,
                Internal_Matrix_Param internal_param,
                External_Energy_Param external_param, GVF_Param gvf_param,
                Snake_Move_Param snake_move_param);
    void run();

   private:
    void initial_contour();
    void calcu_external_force_image();
    void calcu_gvf();
    void calcu_ballon_force();
    void calcu_internal_force_matrix();
    void update_snake();

    Contour contour;
    Internal_Matrix_Param internal_param;
    External_Energy_Param external_param;
    GVF_Param gvf_param;
    Snake_Move_Param snake_move_param;
};