#pragma once

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    // Add public variables or methods declarations here.
    float GetDistance() { return distance; }
    void AStarSearch();

  private:
    // Add private variables or methods declarations here.
    RouteModel &m_Model;
    RouteModel::Node *start_node;
    RouteModel::Node *end_node;
    float distance;
    vector<RouteModel::Node*> open_list;
    RouteModel::Node *NextNode();
    vector <RouteModel::Node> ConstructFinalPath (RouteModel::Node *currentNode);
    float CalculateHValue(RouteModel::Node *node);
    void AddNeighbors(RouteModel::Node *current_node);
};
