#include "data_base.h"
#include "mean_shift.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_COLOR);
    cv::resize(img, img, cv::Size(100, 100));
    std::unique_ptr<DataBase> db_ptr = std::make_unique<ColorData>(img, 100);
    MeanShift ms(db_ptr);
    ms.run();
    cv::waitKey(0);
    return 0;
}
