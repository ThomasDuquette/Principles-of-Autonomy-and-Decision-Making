#pragma once
#include "task.hpp"
#include <iostream>
#include <vector>

struct Node {
    Node* parent;
    int n_affected_tasks = 0;
    int id = 0;
    float path_cost = 0;
    float total_cost = 0;
};

void insert_node(std::list<Node>& node_list, const Node& new_node) {
    auto it = node_list.begin();
    for (; it != node_list.end(); it++) {
        if (new_node.total_cost <= it->total_cost) {
            node_list.insert(it, new_node);
            return;
        }
    }
    // if highest cost
    node_list.push_back(new_node);
}

Array extract_solution(const Node& node) {
    Array result(node.n_affected_tasks);
    auto current_node = node;
    for (int i = node.n_affected_tasks-1; i > 0; i--) {
        result[i] = current_node.id;
        current_node = *current_node.parent;
    }
    result[0] = current_node.id;
    return result;
}

bool is_consistent(Task task, Node node) {
    auto candidate = extract_solution(node);

    auto order_constraints = task.order_constraints;
    for (int i = 0; i < candidate.size; i++) {
        // Domain constraints
        if (task.task_domain(candidate[i], i) == 0) {
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
    
    std::list<Node> active_nodes;
    std::list<Node> parent_list;
    Node original_node;
    original_node.path_cost = 0;

    parent_list.push_back(original_node);
    
    // Populate active nodes with task from initial position to beginning of every task
    real total_cost = 0;
    for (int i = 0; i < task.joint_space_tasks.size(); i++) {
        total_cost += task.minimum_cost_to_reach[i];
    }

    Node new_node;
    new_node.n_affected_tasks = 1;
    new_node.parent = &parent_list.back();
    for (int i = 0; i < task.joint_space_tasks.size(); i++) {
        new_node.id = i;
        new_node.path_cost = task.cost_from_start[i];
        new_node.total_cost = total_cost - new_node.path_cost;
        insert_node(active_nodes, new_node);
    }

    
    while (true) {
        // STOPPING CRITERIA: If no more active nodes, that means no solutions are possible
        if (active_nodes.size() == 0) {
            *success = false;
            return {};
        }
        
        // Expand lowest value node (first in line)
        auto current_node = active_nodes.front();
        
        // STOPPING CRITERIA: if current node is end node, that means we found optimal solution
        if (current_node.n_affected_tasks == n_tasks) {
            *success = true;
            return extract_solution(current_node);
        }
        
        current_node = active_nodes.front();
        // Find currently available nodes
        Array remaining_nodes(n_tasks, 1);
        while (current_node.path_cost != 0) {
            remaining_nodes[current_node.id] = 0;
            current_node = *current_node.parent;
        }

        // Find max cost to go from nodes to end (affecting all tasks)
        total_cost = 0;
        for (int i = 0; i < remaining_nodes.size; i++) {
            if (remaining_nodes[i] == 1) {
                total_cost += task.minimum_cost_to_reach[i];
            }
        }

        // Move current node to parent list
        auto current_it = active_nodes.begin();
        parent_list.splice(parent_list.end(), active_nodes, current_it); // Moves node from active_nodes to parent_list

        // Add new nodes to active nodes
        new_node.parent = &parent_list.back();
        new_node.n_affected_tasks = parent_list.back().n_affected_tasks + 1;
        for (int i = 0; i < remaining_nodes.size; i++) {
            if (remaining_nodes[i] == 1) {
                new_node.id = i;
                if (is_consistent(task, new_node)) { // consistency is not affected by id, path cost or total cost
                    new_node.path_cost = parent_list.back().path_cost + task.cost(parent_list.back().id, i);
                    new_node.total_cost = new_node.path_cost + total_cost - task.minimum_cost_to_reach[i];
                    insert_node(active_nodes, new_node);
                }
            }
        }
    }
}
