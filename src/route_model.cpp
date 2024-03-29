#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    int count = 0;
    for (Model::Node node : this->Nodes()) {
        m_Nodes.push_back(Node(count, this, node));;
        count++;
    }

    CreateNodeToRoadHashmap();
}

void RouteModel::CreateNodeToRoadHashmap() {
    for (const Model::Road &road : Roads()) {
        if (road.type != Model::Road::Type::Footway) {
            for (int node_idx : Ways()[road.way].nodes) {
                if (node_to_road.find(node_idx) == node_to_road.end()) {
                    node_to_road[node_idx] = vector<const Model::Road *>();
                }

                node_to_road[node_idx].push_back(&road);
            }
        }
    }
}

RouteModel::Node &RouteModel::FindClosestNode(float x, float y) {
    Node input;
    input.x = x;
    input.y = y;

    float min_dist = std::numeric_limits<float>::max();
    int closest_idx;
    float temp_distance;

    for(const Road &road : Roads()) {
        if(road.type != Model::Road::Type::Footway) {
            for(const int node_idx : Ways()[road.way].nodes) {
                temp_distance = input.distance(SNodes()[node_idx]);
                if(temp_distance < min_dist) {
                    min_dist = temp_distance;
                    closest_idx = node_idx;
                }
            }
        }
    }

    return SNodes()[closest_idx];
}

RouteModel::Node *RouteModel::Node::FindNeighbor(vector<int> node_indices) {
    Node *closest_node = nullptr;
    Node node;
    float distance;
    for (int node_index : node_indices) {
        node = parent_model->SNodes()[node_index];
        distance = this->distance(node);

        if (distance != 0 && !node.visited) {
            if (closest_node == nullptr || distance < this->distance(*closest_node)) {
                closest_node = &parent_model->SNodes()[node_index];
            }
        }
    }
    return closest_node;
}

void RouteModel::Node::FindNeighbors() {
    for (auto &road : parent_model->node_to_road[this->index]) {
        RouteModel::Node *newNeighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
        if (newNeighbor) {
                    this->neighbors.emplace_back(newNeighbor);
        }
    }
}