#include "data_base.h"
#include "mean_shift.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>

int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_COLOR);
    std::shared_ptr<Visualizer> vis_ptr(new Visualizer);
    std::thread vis_thread(&Visualizer::show, std::ref(*vis_ptr), img.rows, img.cols);
    std::unique_ptr<DataBase> db_ptr = std::make_unique<ColorData>(img, 50, vis_ptr);
    MeanShift ms(db_ptr);

    ms.run();
    vis_thread.join();

    return 0;
}
