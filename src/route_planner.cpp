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


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float distance;
    distance = node->distance(*end_node);
    return distance;
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (RouteModel::Node *n : current_node->neighbors) {
        n->parent = current_node;
        n->g_value += current_node->distance(*n);
        n->h_value = CalculateHValue(n);
        open_list.emplace_back(n);
        n->visited = true;
        std::cout << "the next neighbor g-value is: " << n->g_value << "\n";
        std::cout << "the next neighbor h-value is: " << n->h_value << "\n";
    }

}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    sort (open_list.begin(), open_list.end(), [](const RouteModel::Node *lhs, const RouteModel::Node *rhs) {
        RouteModel::Node left = *lhs;
        RouteModel::Node right = *rhs;
        return left.g_plus_h < right.g_plus_h;
    });
    
    RouteModel::Node *next_node = open_list[0];
    std::cout << "the gh value is" << next_node->g_plus_h << "\n";
    return next_node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    // push nodes onto path found and then reverse vector

    RouteModel::Node curr_parent = *current_node->parent;
    while (current_node->parent != nullptr) {
        float dist_to_parent;
        dist_to_parent = current_node->distance(curr_parent);
        distance += dist_to_parent;
        path_found.insert(path_found.begin(), *current_node);
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    while (current_node != end_node) {
        AddNeighbors(current_node);
        RouteModel::Node *next_node = NextNode();
    }
    m_Model.path = ConstructFinalPath(current_node);

}