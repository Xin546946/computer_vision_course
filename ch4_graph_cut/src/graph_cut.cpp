#include "graph_cut.h"
GraphCut::GraphCut(cv::Mat img)
    : interaction_tool_(img),
      graph_(img, interaction_tool_.get_points_foreground(),
             interaction_tool_.get_points_background()),
      img_(img) {
}

void GraphCut::run() {
    compute_max_flow();
    compute_min_cut();
    segmention_bfs();
}
cv::Mat GraphCut::get_segmentation(SegType type) const {
    cv::Mat result;
    switch (type) {
        case SegType::FOREGROUND:
            /* code */
            break;
        case SegType::BACKGOUND:
            /* code */
            break;

        default:
            break;
    }
    return result;
}