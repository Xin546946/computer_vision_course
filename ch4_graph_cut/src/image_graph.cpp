#include "image_graph.h"
#include <vector>
inline int pos_to_id(int row, int col, int step) {
    return row * step + col + 1;
}

ImageGraph::ImageGraph(cv::Mat img)
    : Graph(img.rows * img.cols + 2), img_(img) {
    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            // build edges from img to s
            add_unary_edge(0, pos_to_id(r, c, img.rows), edge_weight);
            // build edges from img to t
            add_unary_edge(pos_to_id(r, c, img.rows), img.rows * img.cols + 1,
                           edge_weight);
            // build edges through different pixels
            for (int i = 0; i < 4; i++) {
                if (r == 0 && c = 0) {
                    add_binary_edge(pos_to_id(r, c, img.rows),
                                    pos_to_id(r + 1, c, img.rows), edge_weight);
                    add
                }
            }
        }
    }
}
