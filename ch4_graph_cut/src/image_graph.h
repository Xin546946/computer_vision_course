#pragma once
#include "graph.h"
#include <opencv2/core.hpp>

class Edge : public EdgeBase {
   public:
    Edge(double weight);

   private:
    double cap_;
    double flow_;
};

typedef NodeBase<Edge> Node;
class ImageGraph : public Graph<Node, Edge> {
   public:
    ImageGraph(cv::Mat img);

   private:
    cv::Mat img_;

    struct Dist {
        Dist() = default;
        double get_weight(int r, int c, int flag) {
            return 0.0f;
        }

        double compute_weight(cv ::Vec3f, cv::Vec3f) {
            return 0.0f;
        }
    } dist;
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