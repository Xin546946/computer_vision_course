#include "image_graph.h"
#include "distribution.h"
#include "opencv_utils.h"
#include <vector>

static const int dire[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

ImageGraph::ImageGraph(cv::Mat img,
                       const std::vector<cv::Point>& points_foreground,
                       const std::vector<cv::Point>& points_background)
    : Graph(img.rows * img.cols + 2),
      src_id_(0),
      sink_id_(img.rows * img.cols + 1),
      img_(img),
      dist_(img, points_foreground, points_background) {
    cv::Mat w_fore = dist_.get_probability_map(0);
    cv::Mat w_back = dist_.get_probability_map(1);

    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            int id = pos_to_id(r, c, img.cols);
            // build edges from img to s
            add_unary_edge(0, id + 1, Edge(w_fore.at<double>(r, c)));
            // build edges from img to t
            add_unary_edge(id + 1, img.rows * img.cols + 1,
                           Edge(w_back.at<double>(r, c)));
            // build edges through different pixels
            for (int i = 0; i < 4; i++) {
                int r_curr = r + dire[i][0];
                int c_curr = c + dire[i][1];
                if (!is_in_img(img, r_curr, c_curr)) {
                    continue;
                }
                add_unary_edge(id + 1, pos_to_id(r_curr, c_curr, img.cols) + 1,
                               Edge(compute_weight(
                                   img.at<cv::Vec3f>(r, c),
                                   img.at<cv::Vec3f>(r_curr, c_curr), 10)));
            }
        }
    }

    // change wieghts of scribles with src
    auto src_neighbors = nodes_[0].neighbours_;
    for (int i = 0; i < points_foreground.size(); i++) {
        int id =
            pos_to_id(points_foreground[i].y, points_foreground[i].x, img.cols);
        src_neighbors[i].second = Edge(1e5);
    }

    for (int i = 0; i < points_background.size(); i++) {
        int id =
            pos_to_id(points_background[i].y, points_background[i].x, img.cols);
        src_neighbors[i].second = Edge(0);
    }
    // change weights of scribles with target
    for (int i = 0; i < points_foreground.size(); i++) {
        int id = pos_to_id(points_foreground[i].y, points_foreground[i].x,
                           img.cols) +
                 1;
        nodes_[id].neighbours_[0].second = Edge(0.0);
    }

    for (int i = 0; i < points_background.size(); i++) {
        int id = pos_to_id(points_background[i].y, points_background[i].x,
                           img.cols) +
                 1;
        nodes_[id].neighbours_[0].second = Edge(1e5);
    }
}

/*--------------------------------------------------------
#####################implementation: Edge #####################
---------------------------------------------------------*/
Edge::Edge(double weight) : EdgeBase(), cap_(weight), flow_(0.0){};

//! remove assert after testing
double Edge::get_residual() const {
    return cap_ - flow_;
}

bool Edge::is_full() {
    // if (get_residual() <= -1e-10) {
    //    std::cerr << "residual " << get_residual();
    //    assert(get_residual() >= -1e-10);
    //}
    return std::abs(get_residual()) <= 1e-10;
}