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

Array extract_solution_finished(const TaskSequencingProblem task, const Node& node, std::vector<std::vector<Array>>& joint_space_solution) {
    Array result(node.n_affected_tasks);
    joint_space_solution.resize(task.joint_space_tasks.size() + task.cartesian_space_tasks.size());
    auto current_node = node;
    for (int i = node.n_affected_tasks-1; i > 0; i--) {
        result[i] = current_node.id;
        current_node = *current_node.parent;
    }
    result[0] = current_node.id;

    for (int i = 0; i < result.size; i++) {
        joint_space_solution[i].resize(2);
        // If not cartesian task
        if (!is_close(task.working_set[result[i]].start_position, task.working_set[result[i]].end_position)) {
            joint_space_solution[i][0] = task.working_set[result[i]].start_position;
            joint_space_solution[i][1] = task.working_set[result[i]].end_position;
        } else { // cartesian task, take next end position
            joint_space_solution[i][0] = task.working_set[result[i]].start_position;
            joint_space_solution[i][1] = task.working_set[result[i+1]].end_position;
            i++;
        }
    }

    return result;
}

bool is_consistent(TaskSequencingProblem task, Node node) {
    auto candidate = extract_solution(node);

    auto order_constraints = task.order_constraints;
    auto following_constraints = task.following_constraints;
    following_constraints.insert(following_constraints.end(), task.phantom_following_constraints.begin(), task.phantom_following_constraints.end());
    // auto mutual_exclusion_constraints = task.mutual_exclusion_constraints;
    for (int i = 0; i < candidate.size - 1; i++) {
        // Domain constraints
        if (task.task_domain(task.working_set[candidate[i]].task_id, i) == 0) {
            return false;
        }

        // Order constraints
        for (int j = 0; j < order_constraints.size(); j++) {
            if (task.working_set[candidate[i]].task_id == task.order_constraints[j].earlier) {
                order_constraints.erase(order_constraints.begin() + j);
            } else if (task.working_set[candidate[i]].task_id == task.order_constraints[j].later) {
                return false;
            }
        }

        // Following constraints
        for (int j = 0; j < following_constraints.size(); j++) {
            if (task.working_set[candidate[i]].task_id == following_constraints[j].earlier && task.working_set[candidate[i+1]].task_id == following_constraints[j].later) {
                following_constraints.erase(following_constraints.begin() + j);
            } else if (task.working_set[candidate[i]].task_id == following_constraints[j].later) {
                return false;
            }
        }

        // Mutual exclusion constraints
        // for (int j = 0; j < mutual_exclusion_constraints.size(); j++) {
        //     for (int k = i+1; k < candidate.size; k++) {
        //         if ((candidate[i] == mutual_exclusion_constraints[j].task_id_1 && candidate[j] != mutual_exclusion_constraints[j].task_id_2) && 
        //             (candidate[i] == mutual_exclusion_constraints[j].task_id_2 && candidate[j] != mutual_exclusion_constraints[j].task_id_1)) {
        //             return false;
        //         }
        //     }
        // }
        
        // Same task id is automatically mutually excluded
        // note: Only happens in cartesian tasks
        for (int k = i+1; k < candidate.size; k++) {
            if (task.working_set[candidate[i]].task_id == task.working_set[candidate[k]].task_id) {
                return false;
            }
        }
    }

    // Constraints on last point
    // Domain constraints
    if (task.task_domain(candidate[candidate.size-1], candidate.size-1) == 0) {
        return false;
    }

    // Ordering constraints
    for (int j = 0; j < order_constraints.size(); j++) {
        if (candidate[candidate.size-1] == order_constraints[j].later) {
            return false;
        }
    }

    // Following constraints
    for (int j = 0; j < following_constraints.size(); j++) {
        if (candidate[candidate.size-1] == following_constraints[j].later) {
            return false;
        }
    }

    return true;
}

Array A_star(TaskSequencingProblem task, bool* success, std::vector<std::vector<Array>>* joint_space_solution = nullptr) {
    int n_tasks = task.working_set.size();
    int n_clusters = task.joint_space_tasks.size() + 2*task.cartesian_space_tasks.size();
    
    std::list<Node> active_nodes;
    std::list<Node> parent_list;
    Node original_node;
    original_node.path_cost = 0;

    parent_list.push_back(original_node);
    
    // Populate active nodes with task from initial position to beginning of every task
    real total_cost = 0;
    for (int i = 0; i < task.working_set.size(); i++) {
        total_cost += task.minimum_cost_to_reach[i];
    }

    Node new_node;
    new_node.n_affected_tasks = 1;
    new_node.parent = &parent_list.back();
    for (int i = 0; i < task.working_set.size(); i++) {
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
        if (current_node.n_affected_tasks == n_clusters) {
            *success = true;
            if (joint_space_solution) {
                return extract_solution_finished(task, current_node, *joint_space_solution);
            } else {
                return extract_solution(current_node);
            }
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
