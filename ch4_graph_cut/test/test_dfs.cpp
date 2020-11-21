#include "ek.h"
#include "graph_search.h"
int main(int argc, char** argv) {
    int n = 11;     // n is the number of nodes including the src and the sink
    int s = n - 2;  // We define node 9 as source
    int t = n - 1;  // We define node 11 as target

    EKSolver solver(n, s, t);

    // Edges from source
    solver.add_edge(s, 0, 5);
    solver.add_edge(s, 1, 10);
    solver.add_edge(s, 2, 5);

    // Middle edges
    solver.add_edge(0, 3, 10);
    solver.add_edge(1, 0, 15);
    solver.add_edge(1, 4, 20);
    solver.add_edge(2, 5, 10);
    solver.add_edge(3, 4, 25);
    solver.add_edge(3, 6, 10);
    solver.add_edge(4, 2, 5);
    solver.add_edge(4, 7, 30);
    solver.add_edge(5, 7, 5);
    solver.add_edge(5, 8, 10);
    solver.add_edge(7, 3, 15);
    solver.add_edge(7, 8, 5);

    // Edge to sink
    solver.add_edge(6, t, 5);
    solver.add_edge(7, t, 15);
    solver.add_edge(8, t, 25);

    Node* root = solver.get_graph();
    std::vector<bool> visited(n, false);
    // DFS(root, visited);
    BFS(root);

    return 0;
}