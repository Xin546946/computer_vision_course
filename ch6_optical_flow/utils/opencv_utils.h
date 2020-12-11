#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

inline bool is_in_img(cv::Mat img, int row, int col) {
    return row < img.rows && row >= 0 && col < img.cols && col >= 0;
}

inline int pos_to_id(int row, int col, int step) {
    return row * step + col;
}

inline cv::Point id_to_pos(int id, int step) {
    return {id / step, id % step};
}

std::vector<cv::Mat> record_webcam();

cv::Point2i template_matching(cv::Mat img, cv::Mat temp);