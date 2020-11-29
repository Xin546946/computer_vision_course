#pragma once
#include "distribution.h"
#include "graph.h"
#include <opencv2/core.hpp>

struct Edge : public EdgeBase {
    Edge(double weight);
    double get_residual() const;
    bool is_full();
    double cap_;
    double flow_;
};

typedef NodeBase<Edge> Node;
class ImageGraph : public Graph<Node, Edge> {
   public:
    ImageGraph(cv::Mat img, const std::vector<cv::Point>& points_foreground,
               const std::vector<cv::Point>& points_background);

    // todo add ~ImageGraph();
    // typedef Node Node;
    // typedef Edge Edge;
    const int src_id_;
    const int sink_id_;

   private:
    cv::Mat img_;
    Distribution dist_;
};

// todo ignore src version
inline int pos_to_id(int row, int col, int step) {
    return row * step + col;
}
/**
 * @brief return {row col}
 *
 * @param id
 * @param step : img cols
 * @return std::pair<int, int>
 */
inline std::pair<int, int> id_to_pos(int id, int step) {
    return {id / step, id % step};
}

/*--------------------------------------------------------
#####################implementation: Edge #####################
---------------------------------------------------------*/

//! remove assert after testing
inline double Edge::get_residual() const {
    return cap_ - flow_;
}

inline bool Edge::is_full() {
    // assert(get_residual() >= -1e-10);
    return std::abs(get_residual()) <= 1e-20;
}