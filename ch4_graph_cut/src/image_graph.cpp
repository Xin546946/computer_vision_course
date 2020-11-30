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
    // todo build a weighted graph for max flow algorithm.
}

/*--------------------------------------------------------
#####################implementation: Edge #####################
---------------------------------------------------------*/
Edge::Edge(double weight) : EdgeBase(), cap_(weight), flow_(0.0) {
}
