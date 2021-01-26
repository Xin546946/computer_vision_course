#include "math_utils.h"
#include "opencv_utils.h"
#include "visualizer.h"
#include <array>
#include <iostream>
#include <limits>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <random>

int main(int argc, char** argv) {
    DataSet2D data_set = generate_data_set_2d(100, 1.0, 0.0, 5.0, 10);
    double length = std::max(data_set.max_x_ - data_set.min_x_, data_set.max_y_ - data_set.min_y_) / 2;
    length *= 1.2;

    Visualizer vis(-length, length, -length, length);
    for (int i = 0; i < data_set.points_.size(); i++) {
        vis.add_point(data_set.points_[i].x, data_set.points_[i].y, data_set.is_inlier_[i]);
    }

    vis.show(0);

    return 0;
}
