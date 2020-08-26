#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float distance;
    distance = node->distance(*end_node);
    return distance;
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (RouteModel::Node *n : current_node->neighbors) {
        n->parent = current_node;
        n->g_value = current_node->g_value + current_node->distance(*n);
        n->h_value = CalculateHValue(n);
        n->visited = true;
        open_list.emplace_back(n);
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    std::sort (open_list.begin(), open_list.end(), [](const RouteModel::Node *lhs, const RouteModel::Node *rhs) {
        return lhs->g_value + lhs->h_value < rhs->g_value + rhs->h_value;
    });
    RouteModel::Node* next_node = open_list[0];
    open_list.erase(open_list.begin());
    return next_node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    // push nodes onto path found and then reverse vector

    while (current_node->parent != nullptr) {
        float dist_to_parent = current_node->distance(*current_node->parent);
        distance += dist_to_parent;
        path_found.push_back(*current_node);
        current_node = current_node->parent;;
    }

    path_found.push_back(*current_node);
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node* current_node = start_node;
    current_node->visited = true;
    open_list.push_back(start_node);
    
    while (open_list.size()>0) {
        AddNeighbors(current_node);
        current_node = NextNode();
        if (current_node->h_value == 0) {
            break;
        }
    }
    m_Model.path = ConstructFinalPath(current_node);
}
