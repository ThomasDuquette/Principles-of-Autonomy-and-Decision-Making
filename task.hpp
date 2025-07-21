#pragma once
#include "blast_rush.h"

using namespace blast;

enum ConstraintType {
    order,
    // time,
    positive_domain,
    negative_domain
};

struct Constraint {
    ConstraintType type;

    int parent = 0;
    real t_start[2] = {0, 0};
};

struct OrderConstraint {
    int earlier = 0;
    int later = 0;
};

struct PositiveDomainConstraint {
    int task_id = 0;
    Array positive_space = {}; // Domain must be within these values
};

struct NegativeDomainConstraint {
    int task_id = 0;
    Array negative_space = {}; // Domain must be within these values
};

struct JointTask {
    Array start_position = {0};
    Array start_velocity = {0};
    Array start_acceleration = {0};
    Array end_position = {0};
    Array end_velocity = {0};
    Array end_acceleration = {0};
};

// struct CartesianTask {
//     Array start_pose = {0};
//     Array end_pose = {0};
// };

inline real trapezoidal_velocity_profile_time(Matrix task, GenericManipulator manip) {
    Array times(manip.joints);

    for (int i = 0; i < manip.joints; i++) {
        real start_pos = task(i, 0);
        real end_pos = task(i, 3);

        real task_displacement = end_pos - start_pos;

        real top_speed = sign(task_displacement)*sqrt(2*std::abs(task_displacement)/(std::abs(1/manip.amax[i]) + std::abs(1/manip.amin[i])));

        if (top_speed > manip.vmax[i]) {
            real t1 = std::abs(manip.vmax[i] / manip.amax[i]);
            real t2 = std::abs(manip.vmax[i] / manip.amin[i]);
            
            real d1 = std::abs(0.5*manip.amax[i] * t1 * t1);
            real d2 = std::abs(0.5*manip.amin[i] * t2 * t2);

            real d_mid = task_displacement - d1 - d2;

            real t_mid = d_mid / manip.vmax[i];
            times[i] = t_mid + t1 + t2;
        } else if (top_speed < manip.vmin[i]) {
            real t1 = std::abs(manip.vmin[i] / manip.amin[i]);
            real t2 = std::abs(manip.vmin[i] / manip.amax[i]);
            
            real d1 = std::abs(0.5*manip.amin[i] * t1 * t1);
            real d2 = std::abs(0.5*manip.amax[i] * t2 * t2);

            real d_mid = task_displacement + d1 + d2;

            real t_mid = d_mid / manip.vmin[i];
            times[i] = t_mid + t1 + t2;
        } else {
            real t1 = std::abs(top_speed / manip.amax[i]);
            real t2 = std::abs(top_speed / manip.amin[i]);
            times[i] = t1 + t2;
        }
    }

    return max(times);
}

struct Task {
    std::vector<OrderConstraint> order_constraints;
    // std::vector<PositiveDomainConstraint> positive_domain_constraints;
    std::vector<NegativeDomainConstraint> negative_domain_constraints;

    std::vector<JointTask> joint_space_tasks;
    
    Matrix task_domain = {};

    GenericManipulator manip;
    Array start_position = {};

    Array cost_from_start = {};
    Matrix cost = {};
    Array minimum_cost_to_reach = {};

    void add_joint_space_task(const Array& start_position, const Array& end_position, const Array& start_velocity = {}, const Array& end_velocity = {}, const Array& start_acceleration = {}, const Array& end_acceleration = {}) {
        Assert(start_position.size == end_position.size);
        JointTask new_task;
        new_task.start_position = start_position;
        new_task.end_position = end_position;
        if (start_velocity.size == 0) {
            new_task.start_velocity = Array(start_position.size);
        }
        if (end_velocity.size == 0) {
            new_task.end_velocity = Array(start_position.size);
        }
        if (start_acceleration.size == 0) {
            new_task.start_acceleration = Array(start_position.size);
        }
        if (end_acceleration.size == 0) {
            new_task.end_acceleration = Array(start_position.size);
        }
        joint_space_tasks.push_back(new_task);
    }

    void add_order_constraint(const int earlier_task, const int later_task) {
        // Not using Assert() because user will very likely be using this function and the error message is important.
        if (earlier_task >= joint_space_tasks.size()) {
            std::cerr << "Error : Task id " << earlier_task << " is undefined" << std::endl;
        } else if (later_task >= joint_space_tasks.size()) {
            std::cerr << "Error : Task id " << later_task << " is undefined" << std::endl;
        }

        OrderConstraint new_constraint;
        new_constraint.earlier = earlier_task;
        new_constraint.later = later_task;
        order_constraints.push_back(new_constraint);
    }

    // Constrains the domain to a specific list of possible positions.
    // void add_domain_constraint_positive(const int task_id, const Array& positive_domain) {
    //     PositiveDomainConstraint new_constraint;
    //     new_constraint.task_id = task_id;
    //     new_constraint.positive_space = positive_domain;
    //     positive_domain_constraints.push_back(new_constraint);
    // }
    
    // Constrains the domain to a specific list of impossible positions.
    void add_domain_constraint_positive(const int task_id, const Array& negative_domain) {
        NegativeDomainConstraint new_constraint;
        new_constraint.task_id = task_id;
        new_constraint.negative_space = negative_domain;
        negative_domain_constraints.push_back(new_constraint);
    }

    void setup() {
        int n_joints = manip.joints;
        int n_tasks = joint_space_tasks.size();
        cost.resize(n_tasks, n_tasks);
        cost_from_start.resize(n_tasks);
        minimum_cost_to_reach.resize(n_tasks);

        Matrix current_task(manip.joints, 6);

        // Evaluate the cost matrix and the minimum cost to reach, which is used in the h() value (optimal cost estimate from a specific state)
        for (int i = 0; i < n_tasks; i++) {
            // Assert task is the right size for the manipulator
            // Not formulated as Assert() because this function will be used and feedback is important
            if (joint_space_tasks[i].start_position.size != n_joints || 
                joint_space_tasks[i].end_position.size != n_joints || 
                joint_space_tasks[i].start_velocity.size != n_joints || 
                joint_space_tasks[i].end_velocity.size != n_joints || 
                joint_space_tasks[i].start_acceleration.size != n_joints || 
                joint_space_tasks[i].end_acceleration.size != n_joints) {
                    std::cerr << "Task id " << i << "contains at least one parameter (pos, vel, acc) inconsistent with manipulator number of joints" << std::endl;
                }

            // evaluate cost from start position
            for (int j = 0; j < current_task.cols; j++) {
                current_task(j, 0) = start_position[j];
                current_task(j, 2) = joint_space_tasks[i].start_position[j];
            }

            cost_from_start[i] = trapezoidal_velocity_profile_time(current_task, manip);

            for (int j = 0; j < current_task.cols; j++) {
                current_task(j, 0) = joint_space_tasks[i].end_position[j];
            }
            real current_min_cost = INF_REAL;
            for (int j = 0; j < n_tasks; j++) {
                if (i == j) {
                    cost(j, i) = 0;
                } else {
                    // Copy task in matrix form
                    for (int r = 0; r < current_task.cols; r++) {
                        current_task(r, 2) = joint_space_tasks[j].start_position[r];
                    }
                    cost(j, i) = trapezoidal_velocity_profile_time(current_task, manip);
                    current_min_cost = cost(j, i) < current_min_cost ? cost(j, i) : current_min_cost;
                }
            }
            minimum_cost_to_reach[i] = current_min_cost;
        }
        
        // Constrict domains according to domain constraints
        task_domain.resize(n_tasks, n_tasks);

        for (int i = 0; i < task_domain.rows; i++) {
            for (int j = 0; j < task_domain.cols; j++) {
                task_domain(i, j) = 1;
            }
        }

        for (int i = 0; i < negative_domain_constraints.size(); i++) {
            for (int j = 0; j < negative_domain_constraints[i].negative_space.size; j++) {
                task_domain(negative_domain_constraints[i].task_id, j) = 0;
            }
        }
    }

    real evaluate_h(const Array& remaining_tasks) {
        real result = 0;
        
        for (int i = 0; i < remaining_tasks.size; i++) {
            result += minimum_cost_to_reach[remaining_tasks[i]];
        }
        return result;
    }
}
