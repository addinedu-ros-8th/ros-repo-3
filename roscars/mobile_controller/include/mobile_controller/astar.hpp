#ifndef ASTAR_HPP_
#define ASTAR_HPP_
#include "mobile_controller/astar.hpp"

#include <vector>
#include <unordered_map>
#include <limits>
#include <queue>
#include <cmath>

struct Node {
  int id;
  double x, y;
};

struct Edge {
  int from, to;
  double cost;
};

class AStar {
public:
  AStar(const std::vector<Node>& nodes, const std::vector<Edge>& edges);
  std::vector<int> search(int start_id, int goal_id);

private:
  std::unordered_map<int, Node> nodes_;
  std::unordered_map<int, std::vector<std::pair<int,double>>> adj_;

  double heuristic(int a, int b);
};

#endif // ASTAR_HPP_