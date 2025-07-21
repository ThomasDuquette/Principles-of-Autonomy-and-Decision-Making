#pragma once
#include "blast_rush.h"
#include "task.hpp"

struct Node {
    Node* parent;
    int n_affected_tasks = 0;
    int id = 0;
    float path_cost = 0;
    float total_cost = 0;
};

void insert_node(std::vector<Node> node_vector, Node new_node) {
    for (int i = 0; i < node_vector.size(); i++) {
        if (new_node.path_cost <= node_vector[i].path_cost) {
            node_vector.insert(node_vector.begin() + i, new_node);
            break;
        }
    }
    // if highest cost
    node_vector.push_back(new_node);
}

Array extract_solution(const Node& node) {
    Array result(node.n_affected_tasks);
    auto current_node = node;
    for (int i = 0; i < node.n_affected_tasks; i++) {
        result[i] = current_node.id;
        current_node = *current_node.parent;
    }
    return result;
}

bool is_consistent(Task task, Node node) {
    auto candidate = extract_solution(node);

    auto order_constraints = task.order_constraints;
    for (int i = 0; i < candidate.size; i++) {
        // Domain constraints
        if (task.task_domain(i, candidate[i]) == 0) {
            return false;
        }

        // Order constraints
        for (int j = 0; j < order_constraints.size(); j++) {
            if (candidate[i] == task.order_constraints[j].earlier) {
                order_constraints.erase(order_constraints.begin() + j);
            } else if (candidate[i] == task.order_constraints[j].later) {
                return false;
            }
        }
    }
    return true;
}

Array A_star(Task task, bool* success) {
    int n_tasks = task.joint_space_tasks.size();
    
    std::vector<Node> active_nodes;
    Node original_node;
    original_node.path_cost = 0;
    
    // Populate active nodes with task from initial position to beginning of every task
    Node new_node;
    new_node.n_affected_tasks = 1;
    new_node.parent = &original_node;
    
    for (int i = 0; i < task.joint_space_tasks.size(); i++) {
        new_node.path_cost = task.cost_from_start[i];
        insert_node(active_nodes, new_node);
    }
    
    while (true) {
        // STOPPING CRITERIA: If no more active nodes, that means no solutions are possible
        if (active_nodes.size() == 0) {
            *success = false;
            return {};
        }
        
        // Expand lowest value node (first in line)
        auto current_node = active_nodes[0];
        
        // STOPPING CRITERIA: if current node is end node, that means we found optimal solution
        if (current_node.n_affected_tasks == n_tasks) {
            *success = true;

            // Extract solution
            Array result(n_tasks);
            for (int i = 0; i < n_tasks; i++) {
                result[i] = current_node.id;
                current_node = *current_node.parent;
            }
            return result;
        }
        
        // Find currently available nodes
        Array remaining_nodes(n_tasks, 1);
        while (current_node.path_cost != 0) {
            remaining_nodes[current_node.id] = 0;
            current_node = *current_node.parent;
        }

        // Find max cost to go from nodes to end (affecting all tasks)
        current_node = active_nodes[0];
        real total_cost = 0;
        for (int i = 0; i < remaining_nodes.size; i++) {
            if (remaining_nodes[i] == 1) {
                total_cost += task.minimum_cost_to_reach[i];
            }
        }

        // Add new nodes to active nodes
        new_node.parent = &current_node;
        new_node.n_affected_tasks = current_node.n_affected_tasks + 1;
        for (int i = 0; i < remaining_nodes.size; i++) {
            if (remaining_nodes[i] == 1) {
                if (is_consistent(task, new_node)) { // consistency is not affected by id, path cost or total cost
                    new_node.id = i;
                    new_node.path_cost = current_node.path_cost + task.cost(current_node.id, i);
                    new_node.total_cost = new_node.path_cost + total_cost - task.minimum_cost_to_reach[i];
                    insert_node(active_nodes, new_node);
                }
            }
        }
    }
}
