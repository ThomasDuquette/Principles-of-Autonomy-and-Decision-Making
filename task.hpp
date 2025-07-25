#pragma once
#include "blast_rush.h"
#include <iostream>
#include <vector>

using namespace blast;

enum TaskType {
    joint,
    cartesian
};

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

struct FollowingConstraint {
    int earlier = 0;
    int later = 0;
};

struct DomainConstraint {
    int task_id = 0;
    Array domain = {}; // Domain must be within these values
};

struct MutualExclusionConstraint {
    int task_id_1 = 0;
    int task_id_2 = 0;
};

struct JointTask {
    int task_id;
    Array start_position = {};
    Array start_velocity = {};
    Array start_acceleration = {};
    Array end_position = {};
    Array end_velocity = {};
    Array end_acceleration = {};

    JointTask() = default;

    JointTask(int n_joints) :
        start_position(Array(n_joints)),
        start_velocity(Array(n_joints)),
        start_acceleration(Array(n_joints)),
        end_position(Array(n_joints)),
        end_velocity(Array(n_joints)),
        end_acceleration(Array(n_joints)) {}

    JointTask(Matrix task) {
        int n_joints = task.rows;

        start_position = Array(n_joints);
        start_velocity = Array(n_joints);
        start_acceleration = Array(n_joints);
        end_position = Array(n_joints);
        end_velocity = Array(n_joints);
        end_acceleration = Array(n_joints);

        for (int i = 0; i < n_joints; i++) {
            start_position[i] = task(i, 0);
            start_velocity[i] = task(i, 1);
            start_acceleration[i] = task(i, 2);
            end_position[i] = task(i, 3);
            end_velocity[i] = task(i, 4);
            end_acceleration[i] = task(i, 5);
        }
    }
};

struct CartesianTask {
    Array start_pose = {0, 0, 0, 0, 0, 0};
    Array end_pose = {0, 0, 0, 0, 0, 0};

    std::vector<Array> start_joint_solutions = {};
    std::vector<Array> end_joint_solutions = {};

    // Fills joint_solutions with allowable joint positions
    // todo: implement
    void get_all_IK(const World& world) {
        start_joint_solutions.push_back({0.0, 1.0, 2.0, 3.0, 4.0, 2.0});
        start_joint_solutions.push_back({1.0, 1.0, 2.0, 2.0, 4.0, 5.0});
        start_joint_solutions.push_back({2.0, 1.0, 4.0, 3.0, 6.0, 3.0});
        start_joint_solutions.push_back({0.0, 3.0, 2.0, 5.0, 4.0, 5.0});
        start_joint_solutions.push_back({3.0, 5.0, 2.0, 1.0, 4.0, 2.0});
        end_joint_solutions.push_back({0.5, 1.25, 2.0, 3.0, 6.0, 5.0});
        end_joint_solutions.push_back({5.0, 0.5, 2.25, 2.0, 4.0, 1.0});
        end_joint_solutions.push_back({2.0, 1.0, 0.5, 3.25, 6.0, 5.0});
        end_joint_solutions.push_back({1.0, 3.0, 3.0, 0.5, 4.25, 0.0});
        end_joint_solutions.push_back({3.0, 2.0, 2.0, 1.0, 0.5, 2.25});
    }
};

struct Task {
    TaskType type;
    int task_id;
    CartesianTask cartesian_task;
    JointTask joint_task;

    Task(const JointTask new_task):
        type(TaskType::joint),
        joint_task(new_task) {}
        
    Task(CartesianTask new_task):
        type(TaskType::cartesian), cartesian_task(new_task) {}
};

// todo: Adapt for velocity different than 0
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

// todo: Adapt for velocity different than 0
inline real trapezoidal_velocity_profile_time(JointTask task, GenericManipulator manip) {
    Array times(manip.joints);

    for (int i = 0; i < manip.joints; i++) {
        real start_pos = task.start_position[i];
        real end_pos = task.end_position[i];
        // real start_vel = task.start_velocity[i];
        // real end_vel = task.end_velocity[i];

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

struct TaskSequencingProblem {
    std::vector<OrderConstraint> order_constraints;
    std::vector<DomainConstraint> domain_constraints;
    std::vector<FollowingConstraint> following_constraints;
    std::vector<FollowingConstraint> phantom_following_constraints;

    std::vector<JointTask> working_set = {};
    
    // note: Is only used to fill working_set in setup()
    std::vector<JointTask> joint_space_tasks = {};
    std::vector<CartesianTask> cartesian_space_tasks = {};

    std::vector<Task> tasks = {};
    
    Matrix task_domain = {};

    GenericManipulator manip;
    Array start_position = {};

    Array cost_from_start = {};
    Matrix cost = {};
    Array minimum_cost_to_reach = {};

    TaskSequencingProblem(GenericManipulator new_manip) 
        : manip(new_manip) {}

    private: 
        void add_joint_space_task(const Array& start_position, const Array& end_position, const Array& start_velocity = {}, const Array& end_velocity = {}, const Array& start_acceleration = {}, const Array& end_acceleration = {}) {
            Assert(start_position.size == end_position.size);
            JointTask new_task(start_position.size);
            new_task.start_position = start_position;
            new_task.end_position = end_position;
            if (start_velocity.size == 0) {
                new_task.start_velocity = Array(start_position.size);
            } else {
                new_task.start_velocity = start_velocity;
            }
            if (end_velocity.size == 0) {
                new_task.end_velocity = Array(start_position.size);
            } else {
                new_task.end_velocity = end_velocity;
            }
            if (start_acceleration.size == 0) {
                new_task.start_acceleration = Array(start_position.size);
            } else {
                new_task.start_acceleration = start_acceleration;
            }
            if (end_acceleration.size == 0) {
                new_task.end_acceleration = Array(start_position.size);
            } else {
                new_task.end_acceleration = end_acceleration;
            }
            joint_space_tasks.push_back(new_task);
        }

        void add_joint_space_task(JointTask new_task) {
            Assert(new_task.start_position.size == new_task.end_position.size);

            if (new_task.start_velocity.size == 0) {
                new_task.start_velocity = Array(start_position.size);
            } else {
                new_task.start_velocity = new_task.start_velocity;
            }
            if (new_task.end_velocity.size == 0) {
                new_task.end_velocity = Array(start_position.size);
            } else {
                new_task.end_velocity = new_task.end_velocity;
            }
            if (new_task.start_acceleration.size == 0) {
                new_task.start_acceleration = Array(start_position.size);
            } else {
                new_task.start_acceleration = new_task.start_acceleration;
            }
            if (new_task.end_acceleration.size == 0) {
                new_task.end_acceleration = Array(start_position.size);
            } else {
                new_task.end_acceleration = new_task.end_acceleration;
            }
            joint_space_tasks.push_back(new_task);
        }

        void add_cartesian_space_task(const Array& start_pose, const Array& end_pose) {
            if (start_pose.size != 6 || end_pose.size != 6) {
                std::cerr << "Invalid pose vector size" << std::endl;
            }
            CartesianTask new_task;
            new_task.start_pose = start_pose;
            new_task.end_pose = end_pose;
            cartesian_space_tasks.push_back(new_task);
        }

        void add_cartesian_space_task(const CartesianTask new_task) {
            if (new_task.start_pose.size != 6 || new_task.end_pose.size != 6) {
                std::cerr << "Invalid pose vector size" << std::endl;
            }
            cartesian_space_tasks.push_back(new_task);
        }

        void add_phantom_following_constraint(const int earlier_task, const int later_task) {
            // Not using Assert() because user will very likely be using this function and the error message is important.
            if (earlier_task > working_set.back().task_id) {
                std::cerr << "Error : Task id " << earlier_task << " is undefined" << std::endl;
            } else if (later_task > working_set.back().task_id) {
                std::cerr << "Error : Task id " << later_task << " is undefined" << std::endl;
            }

            FollowingConstraint new_constraint;
            new_constraint.earlier = earlier_task;
            new_constraint.later = later_task;
            phantom_following_constraints.push_back(new_constraint);
        }

    public: 
        void add_task(Task new_task) {
            new_task.task_id = joint_space_tasks.size() + cartesian_space_tasks.size();
            if (new_task.type == TaskType::joint) {
                new_task.joint_task.task_id = new_task.task_id;
                add_joint_space_task(new_task.joint_task);
                new_task.joint_task = joint_space_tasks.back();
                tasks.push_back(new_task);
            } else if (new_task.type == TaskType::cartesian) {
                add_cartesian_space_task(new_task.cartesian_task);
                tasks.push_back(new_task);
            }
        }

        void add_order_constraint(const int earlier_task, const int later_task) {
            // Not using Assert() because user will very likely be using this function and the error message is important.
            if (earlier_task > tasks.back().task_id) {
                std::cerr << "Error : Task id " << earlier_task << " is undefined" << std::endl;
            } else if (later_task > tasks.back().task_id) {
                std::cerr << "Error : Task id " << later_task << " is undefined" << std::endl;
            }

            OrderConstraint new_constraint;
            new_constraint.earlier = earlier_task;
            new_constraint.later = later_task;
            order_constraints.push_back(new_constraint);
        }

        void add_following_constraint(const int earlier_task, const int later_task) {
            // Not using Assert() because user will very likely be using this function and the error message is important.
            if (earlier_task >= tasks.back().task_id) {
                std::cerr << "Error : Task id " << earlier_task << " is undefined" << std::endl;
            } else if (later_task >= tasks.back().task_id) {
                std::cerr << "Error : Task id " << later_task << " is undefined" << std::endl;
            }

            FollowingConstraint new_constraint;
            new_constraint.earlier = earlier_task;
            new_constraint.later = later_task;
            following_constraints.push_back(new_constraint);
        }

        // Specify the domain of a variable to a specific list.
        void add_domain_constraint(const int task_id, const Array& domain) {
            // Not using Assert() because user will very likely be using this function and the error message is important.
            if (task_id >= tasks.size()) {
                std::cerr << "Error : Task id " << task_id << " is undefined" << std::endl;
            }
            DomainConstraint new_constraint;
            new_constraint.task_id = task_id;
            new_constraint.domain = domain;
            domain_constraints.push_back(new_constraint);
        }

        void setup(const World& world) {
            working_set.clear();
            int n_joints = manip.joints;

            // Add tasks to working set
            int idx = tasks.back().task_id+1;
            for (int i = 0; i < tasks.size(); i++) {
                switch (tasks[i].type) {
                    // Simple case: Task is in joint space
                    case TaskType::joint:
                        tasks[i].joint_task.task_id = tasks[i].task_id;
                        working_set.push_back(tasks[i].joint_task);
                        break;
                    // Task is in cartesian space
                    case TaskType::cartesian:
                        tasks[i].cartesian_task.get_all_IK(world);
                        // Add tasks reaching start positions
                        for (int j = 0; j < tasks[i].cartesian_task.start_joint_solutions.size(); j++) {
                            JointTask new_task(tasks[i].cartesian_task.start_joint_solutions[j].size);
                            new_task.start_position = tasks[i].cartesian_task.start_joint_solutions[j];
                            new_task.end_position = tasks[i].cartesian_task.start_joint_solutions[j];
                            new_task.task_id = tasks[i].task_id;
                            working_set.push_back(new_task);
                        }
                        // Add tasks reaching end positions
                        for (int j = 0; j < tasks[i].cartesian_task.end_joint_solutions.size(); j++) {
                            JointTask new_task(tasks[i].cartesian_task.end_joint_solutions[j].size);
                            new_task.start_position = tasks[i].cartesian_task.end_joint_solutions[j];
                            new_task.end_position = tasks[i].cartesian_task.end_joint_solutions[j];
                            new_task.task_id = idx;
                            working_set.push_back(new_task);
                        }
                        // Add following constraint between the two tasks
                        add_phantom_following_constraint(tasks[i].task_id, idx);
                        idx++;
                        // note: Mutual exclusion constraint is not necessary since the tasks have the same task id
                        break;
                    default:
                        std::cerr << "Unrecognized task type" << std::endl;
                        break;
                }
            }

            int n_tasks = working_set.size();

            cost.resize(n_tasks, n_tasks);
            cost_from_start.resize(n_tasks);
            minimum_cost_to_reach.resize(n_tasks);

            Matrix current_task(manip.joints, 6);

            // Evaluate the cost matrix and the minimum cost to reach, which is used in the h() value (optimal cost estimate from a specific state)
            for (int i = 0; i < n_tasks; i++) {
                // Assert task is the right size for the manipulator
                // Not formulated as Assert() because this function will be used and feedback is important
                if (working_set[i].start_position.size != n_joints || 
                    working_set[i].end_position.size != n_joints || 
                    working_set[i].start_velocity.size != n_joints || 
                    working_set[i].end_velocity.size != n_joints || 
                    working_set[i].start_acceleration.size != n_joints || 
                    working_set[i].end_acceleration.size != n_joints) {
                        std::cerr << "Task id " << i << "contains at least one parameter (pos, vel, acc) inconsistent with manipulator number of joints" << std::endl;
                    }

                // Evaluate cost from start position
                for (int j = 0; j < current_task.cols; j++) {
                    current_task(j, 0) = start_position[j];
                    current_task(j, 3) = working_set[i].start_position[j];
                }

                cost_from_start[i] = trapezoidal_velocity_profile_time(current_task, manip);

                // Fill mcost matrix
                for (int j = 0; j < current_task.cols; j++) {
                    current_task(j, 0) = working_set[i].end_position[j];
                }
                real current_min_cost = INF_REAL;
                for (int j = 0; j < n_tasks; j++) {
                    if (working_set[i].task_id == working_set[j].task_id) {
                        cost(j, i) = 0;
                    } else {
                        // Copy task in matrix form
                        for (int r = 0; r < current_task.cols; r++) {
                            current_task(r, 3) = working_set[j].start_position[r];
                        }
                        cost(j, i) = trapezoidal_velocity_profile_time(current_task, manip);
                        current_min_cost = cost(j, i) < current_min_cost ? cost(j, i) : current_min_cost;
                    }
                }
                minimum_cost_to_reach[i] = current_min_cost;
            }
            
            // Constrict domains according to domain constraints
            task_domain.resize(n_tasks, n_tasks);

            // Fill with 1
            for (int i = 0; i < task_domain.rows; i++) {
                for (int j = 0; j < task_domain.cols; j++) {
                    task_domain(i, j) = 1;
                }
            }

            // Replace unallowable domain with 0
            for (int i = 0; i < domain_constraints.size(); i++) {
                for (int j = 0; j < domain_constraints[i].domain.size; j++) {
                    task_domain(domain_constraints[i].task_id, j) = domain_constraints[i].domain[j];
                }
            }
        }
};
