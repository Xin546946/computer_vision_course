#include "image_graph.h"
#include <vector>

inline int pos_to_id(int row, int col, int step) {
    return row * step + col + 1;
}

static int dire[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

ImageGraph::ImageGraph(cv::Mat img)
    : Graph(img.rows * img.cols + 2), img_(img) {
    for (int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            // build edges from img to s
            add_unary_edge(0, pos_to_id(r, c, img.rows), Edge());
            // build edges from img to t
            add_unary_edge(pos_to_id(r, c, img.rows), img.rows * img.cols + 1,
                           edge_weight);
            // build edges through different pixels
            for (int i = 0; i < 4; i++) {
                int r_curr = r + dire[i][0];
                int c_curr = c + dire[i][1];
                if (r_curr < 0 || r_curr >= img.rows || c_curr < 0 ||
                    c_curr >= img.cols) {
                    continue;
                }
                add_unary_edge(pos_to_id(r, c, img.rows),
                               pos_to_id(r_curr, c_curr, img.rows));
            }
        }
    }
}
