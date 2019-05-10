#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &model.FindClosestNode(start_x, start_y);
    end_node = &model.FindClosestNode(end_x, end_y);
}

void RoutePlanner::AStarSearch() {
//    end_node->parent = start_node;
//    m_Model.path = ConstructFinalPath(end_node);
//    return;
    start_node->visited = true;
    open_list.push_back(start_node);
    RouteModel::Node *current_node = nullptr;

    while(open_list.size() > 0) {
        current_node = NextNode();

        if(current_node->distance(*end_node) == 0) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        } else {
            AddNeighbors(current_node);
        }
    }

}

vector <RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *currentNode) {
    vector <RouteModel::Node> path_found {};
    distance = 0.0f;
    RouteModel::Node parent;

    while(currentNode->parent != nullptr) {
        path_found.push_back(*currentNode);
        parent = *(currentNode->parent);
        distance += currentNode->distance(parent);
        currentNode = currentNode->parent;
    }

    path_found.push_back(*currentNode);
    distance *= m_Model.MetricScale();
    return path_found;
}

float RoutePlanner::CalculateHValue(RouteModel::Node *node) {
    return node->distance(*end_node);
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), [](const auto &_1st, const auto &_2nd){
        return _1st->h_value + _1st->g_value < _2nd->h_value + _2nd->g_value;
    });

    RouteModel::Node *lowest_node = open_list.front();
    open_list.erase(open_list.begin());
    return lowest_node;
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for  (RouteModel::Node * node : current_node->neighbors) {
        node->parent = current_node;
        node->g_value = current_node->g_value + current_node->distance(*node);
        node->h_value = CalculateHValue(node);
        open_list.push_back(node);
        node->visited = true;
    }
}