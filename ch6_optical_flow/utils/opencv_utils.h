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
/**
 * @brief get location of a temlate given, image and a template
 *
 * @param img
 * @param temp
 * @return cv::Point2i
 */
cv::Point2i template_matching(cv::Mat img, cv::Mat temp);
/**
 * @brief get a sub image, safe at boundary without autofilling
 *
 * @param image
 * @param x : x of window center
 * @param y : y of window center
 * @param width : widht of the window
 * @param height : height of the window
 * @return cv::Mat : sub im
 */
cv::Mat get_sub_image(cv::Mat image, int x, int y, int width, int height);

cv::Mat get_bounding_box_vis_image(cv::Mat image, int x, int y, int width, int height);

// todo
/**
 * @brief put val to a cv::Mat with up-left(x,y), and window size(widht, height)
 *
 * @tparam T
 * @param val
 * @param input_mat
 * @param x_ul
 * @param y_ul
 * @param width
 * @param height
 * @return cv::Mat
 */
template <typename T>
cv::Mat put_val(T val, cv::Mat input_mat, int x_ul, int y_ul, int width, int height);