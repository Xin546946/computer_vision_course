#include "graph_search.h"
#include "image_graph.h"
#include "interaction_tool.h"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unordered_set>

int main(int argc, char** argv) {
    // cv::Mat img = cv::Mat_<double>(4, 4) << 0, 0, 0, 255, 0, 0, 255, 255, 0,
    // 0,
    //               255, 255, 0, 255, 255, 255);
    // std::cout << img << '\n';
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_COLOR);

    const auto& points = drag_to_get_fore_and_background_scribbles(img);

    ImageGraph graph(img, points[0], points[1]);
    // std::unordered_set<Node*> visited;
    // DFS(graph.get_root(), visited);
    // BFS(graph.get_root());
    BFS(graph.get_root(), img.rows, img.cols);

    return 0;
}