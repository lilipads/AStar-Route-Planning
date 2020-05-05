#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <string>
#include <vector>
#include <queue>

#include "route_model.h"

class RoutePlanner {
 public:
  RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x,
               float end_y);
  float GetDistance() const { return distance; }
  void AStarSearch();

  void AddNeighbors(RouteModel::Node *current_node);
  float CalculateHValue(RouteModel::Node const *node);
  std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *);
  RouteModel::Node *NextNode();

 private:
  struct NodeGreater{
    bool operator()(const RouteModel::Node* lhs, const RouteModel::Node* rhs){
      return lhs->g_value + lhs->h_value > rhs->g_value + rhs->h_value;
    }
  };
  std::priority_queue<RouteModel::Node*, std::vector<RouteModel::Node*>, NodeGreater> open_list;
  RouteModel::Node *start_node;
  RouteModel::Node *end_node;

  float distance = 0.0f;
  RouteModel &m_Model;
};

#endif