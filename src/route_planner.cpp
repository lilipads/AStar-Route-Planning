#include "route_planner.h"

#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model) {
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  /* h value is defined by the distance to the end_node*/

  return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  /* expand the current node by adding all uvisited neighbors to the open_list.
    For each neighbor, define its parent (current node), h value, and g_value
    (distance to origin)
  */

  current_node->FindNeighbors();
  for (auto node : current_node->neighbors) {
    if (!node->visited) {
      node->parent = current_node;
      node->h_value = CalculateHValue(node);
      node->g_value = current_node->g_value + current_node->distance(*node);
      node->visited = true;
      open_list.push(node);
    }
  }
}

RouteModel::Node *RoutePlanner::NextNode() {
  /* pop the node in open_list with the smallest sum of h value and g value and
   * return it
   */

  if (open_list.empty()) {
    throw std::invalid_argument("no valid path exists!");
  }
  RouteModel::Node *smallest_node = open_list.top();
  open_list.pop();
  return smallest_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(
    RouteModel::Node *current_node) {
  /* define the shortest distance to the distance class variable,
    and return the path (starting from the start_node, and ending with the
    end_node).

    params:
     - current_node: the end_node returned by A* algorithm.
  */

  distance = current_node->g_value;
  std::vector<RouteModel::Node> path_found;

  while (current_node != nullptr) {
    path_found.push_back(*current_node);
    current_node = current_node->parent;
  }
  // Multiply the distance by the scale of the map to get meters.
  distance *= m_Model.MetricScale();
  std::reverse(path_found.begin(), path_found.end());
  return path_found;
}

void RoutePlanner::AStarSearch() {
  /*
   use A* algorithm to find shortest path from start_node and end_node.
   path will be stored in m_Model.path,
  and shortest distance will be stored in the class variable distance.
  */

  RouteModel::Node *current_node = start_node;
  start_node->visited = true;

  while (current_node != end_node) {
    AddNeighbors(current_node);
    current_node = NextNode();
  }
  m_Model.path = ConstructFinalPath(current_node);
}