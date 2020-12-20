#include "data_base.h"
#include "mean_shift.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_COLOR);
    cv::resize(img, img, cv::Size(30, 30));
    std::unique_ptr<DataBase> db_ptr = std::make_unique<ColorData>(img, 30);
    MeanShift ms(db_ptr);
    ms.run();

    return 0;
}
