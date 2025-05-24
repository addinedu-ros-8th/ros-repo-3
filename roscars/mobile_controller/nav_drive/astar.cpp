#include "astar.hpp"
#include <algorithm>

AStar::AStar(const std::vector<Node>& nodes, const std::vector<Edge>& edges) {
  for (const auto& n : nodes) {
    nodes_[n.id] = n;
  }
  for (const auto& e : edges) {
    adj_[e.from].push_back({e.to, e.cost});
    adj_[e.to].push_back({e.from, e.cost});
  }
}

double AStar::heuristic(int a, int b) {
  auto& n1 = nodes_[a];
  auto& n2 = nodes_[b];
  double dx = n1.x - n2.x;
  double dy = n1.y - n2.y;
  return std::sqrt(dx*dx + dy*dy);
}

std::vector<int> AStar::search(int start_id, int goal_id) {
  struct State { int id; double f; };
  struct Cmp { bool operator()(State a, State b) const { return a.f > b.f; } };
  std::priority_queue<State, std::vector<State>, Cmp> open;

  std::unordered_map<int, double> g;
  std::unordered_map<int, int> parent;

  for (auto& [id,_] : nodes_) g[id] = std::numeric_limits<double>::infinity();
  g[start_id] = 0;
  open.push({start_id, heuristic(start_id, goal_id)});

  while (!open.empty()) {
    auto cur = open.top(); open.pop();
    if (cur.id == goal_id) break;
    for (auto& [nid, cost] : adj_[cur.id]) {
      double ng = g[cur.id] + cost;
      if (ng < g[nid]) {
        g[nid] = ng;
        parent[nid] = cur.id;
        double f = ng + heuristic(nid, goal_id);
        open.push({nid, f});
      }
    }
  }

  std::vector<int> path;
  int cur = goal_id;
  while (cur != start_id) {
    path.push_back(cur);
    cur = parent[cur];
  }
  path.push_back(start_id);
  std::reverse(path.begin(), path.end());
  return path;
}