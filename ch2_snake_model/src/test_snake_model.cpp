#include "snake_model.cpp"
#include <iostream>

int main(int argc, char** argv) {
    cv::Mat img;
    Contour contour;  // TODO how to initialize a contour
    Internal_Matrix_Param internal_param{0.01f, 0.01f};
    External_Energy_Param external_param{0.04f, 2, 0.01f};
    GVF_Param gvf_param{0.2f, 50, 1};
    Snake_Move_Param snake_move_param{1, 10, 0.01f};
    Snake_Model snake_model(img, contour, internal_param, external_param,
                            snake_move_param);
    snake_model.run();

    return 0;
}