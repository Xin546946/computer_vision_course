#include "graph_search.h"
#include "image_graph.h"
#include <iostream>
#include <opencv2/core.hpp>

int main(int argc, char** argv) {
    cv::Mat img = (cv::Mat_<double>(4, 4) << 0, 0, 0, 255, 0, 0, 255, 255, 0, 0,
                   255, 255, 0, 255, 255, 255);
    std::cout << img << '\n';

    ImageGraph graph(img);
    BFS(graph.get_root());

    return 0;
}