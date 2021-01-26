#include "visualizer.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    Visualizer vis(-100, 100, -100, 100);
    vis.add_point(-20, 20, false);
    vis.add_point(50, 50, true);

    vis.add_line(1, 0, false, 10);
    vis.show(0);

    return 0;
}