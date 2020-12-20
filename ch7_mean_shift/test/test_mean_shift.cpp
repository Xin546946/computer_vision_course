#include "data_base.h"
#include "mean_shift.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_COLOR);
    std::unique_ptr<DataBase> db_ptr = std::make_unique<ColorData>(img, 30);
    MeanShift ms(db_ptr);
    ms.run();

    return 0;
}
