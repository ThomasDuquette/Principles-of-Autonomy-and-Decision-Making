#define CATCH_CONFIG_MAIN
#include "catch2/catch.hpp"
#include "blast_rush.h"
#include "../extern/blast_rush/tests/test_helper/test_helper.hpp"
#include "../extern/blast_rush/tests/test_helper/test_functions.hpp"
#include "../task.hpp"
#include "../utilities.hpp"

using namespace blast;

TEST_CASE("JointTask constructor(Matrix) function test", "[Task]") {
    auto demo1_tasks = get_Link6_demo1_tasks_simple();
    
    JointTask joint_task(demo1_tasks[0]);

    for (int i = 0; i < demo1_tasks[0].rows; i++) {
        CHECK(is_close(joint_task.start_position[i], demo1_tasks[0](i, 0)));
        CHECK(is_close(joint_task.end_position[i], demo1_tasks[0](i, 3)));
        CHECK(is_close(joint_task.start_velocity[i], demo1_tasks[0](i, 1)));
        CHECK(is_close(joint_task.end_velocity[i], demo1_tasks[0](i, 4)));
        CHECK(is_close(joint_task.start_acceleration[i], demo1_tasks[0](i, 2)));
        CHECK(is_close(joint_task.end_acceleration[i], demo1_tasks[0](i, 5)));
    }
}

TEST_CASE("trapezoidal_velocity_profile_time() function test", "[Task]") {
    auto manip = get_generic_Link6();
    auto demo1_tasks = get_Link6_demo1_tasks_simple();
    
    JointTask joint_task(demo1_tasks[0]);

    auto result = trapezoidal_velocity_profile_time(joint_task, manip);
    real expected_result = 0.794814;
    CHECK(is_close(result, expected_result));
}

TEST_CASE("Task struct: add_joint_space_task(Arrays) with fully defined inputs function test", "[Task]") {
    auto manip = get_generic_Link6();
    auto demo1_tasks = get_Link6_demo1_tasks_simple();
    
    JointTask joint_task(demo1_tasks[0]);
    Task example_task(manip);

    example_task.add_joint_space_task(joint_task.start_position, joint_task.end_position, joint_task.start_velocity, joint_task.end_velocity, joint_task.start_acceleration, joint_task.end_acceleration);

    CHECK(is_close(example_task.joint_space_tasks[0].start_position, joint_task.start_position));
    CHECK(is_close(example_task.joint_space_tasks[0].end_position, joint_task.end_position));
    CHECK(is_close(example_task.joint_space_tasks[0].start_velocity, joint_task.start_velocity));
    CHECK(is_close(example_task.joint_space_tasks[0].end_velocity, joint_task.end_velocity));
    CHECK(is_close(example_task.joint_space_tasks[0].start_acceleration, joint_task.start_acceleration));
    CHECK(is_close(example_task.joint_space_tasks[0].end_acceleration, joint_task.end_acceleration));
}

TEST_CASE("Task struct: add_joint_space_task(Arrays) with partially defined inputs function test", "[Task]") {
    auto manip = get_generic_Link6();
    auto demo1_tasks = get_Link6_demo1_tasks_simple();
    
    JointTask joint_task(demo1_tasks[0].rows);

    Task example_task(manip);

    example_task.add_joint_space_task(joint_task.start_position, joint_task.end_position, Array(manip.joints, 1));

    CHECK(is_close(example_task.joint_space_tasks[0].start_position, joint_task.start_position));
    CHECK(is_close(example_task.joint_space_tasks[0].end_position, joint_task.end_position));
    CHECK(is_close(example_task.joint_space_tasks[0].start_velocity, Array(manip.joints, 1)));
    CHECK(is_close(example_task.joint_space_tasks[0].end_velocity, Array(manip.joints)));
    CHECK(is_close(example_task.joint_space_tasks[0].start_acceleration, Array(manip.joints)));
    CHECK(is_close(example_task.joint_space_tasks[0].end_acceleration, Array(manip.joints)));
}

TEST_CASE("Task struct: add_joint_space_task(JointTask) with fully defined inputs function test", "[Task]") {
    auto manip = get_generic_Link6();
    auto demo1_tasks = get_Link6_demo1_tasks_simple();
    
    JointTask joint_task(demo1_tasks[0]);
    Task example_task(manip);

    example_task.add_joint_space_task(joint_task);

    CHECK(is_close(example_task.joint_space_tasks[0].start_position, joint_task.start_position));
    CHECK(is_close(example_task.joint_space_tasks[0].end_position, joint_task.end_position));
    CHECK(is_close(example_task.joint_space_tasks[0].start_velocity, joint_task.start_velocity));
    CHECK(is_close(example_task.joint_space_tasks[0].end_velocity, joint_task.end_velocity));
    CHECK(is_close(example_task.joint_space_tasks[0].start_acceleration, joint_task.start_acceleration));
    CHECK(is_close(example_task.joint_space_tasks[0].end_acceleration, joint_task.end_acceleration));
}

TEST_CASE("Task struct: add_joint_space_task(JointTask) with partially defined inputs function test", "[Task]") {
    auto manip = get_generic_Link6();
    auto demo1_tasks = get_Link6_demo1_tasks_simple();
    
    JointTask joint_task(demo1_tasks[0]);

    Task example_task(manip);

    joint_task.start_velocity = Array(manip.joints, 1);
    joint_task.end_velocity = {};
    joint_task.start_acceleration = {};
    joint_task.end_acceleration = {};

    example_task.add_joint_space_task(joint_task);

    CHECK(is_close(example_task.joint_space_tasks[0].start_position, joint_task.start_position));
    CHECK(is_close(example_task.joint_space_tasks[0].end_position, joint_task.end_position));
    CHECK(is_close(example_task.joint_space_tasks[0].start_velocity, Array(manip.joints, 1)));
    CHECK(is_close(example_task.joint_space_tasks[0].end_velocity, Array(manip.joints)));
    CHECK(is_close(example_task.joint_space_tasks[0].start_acceleration, Array(manip.joints)));
    CHECK(is_close(example_task.joint_space_tasks[0].end_acceleration, Array(manip.joints)));
}

TEST_CASE("Task struct: add_order_constraint() function test", "[Task]") {
    auto manip = get_generic_Link6();
    auto demo1_tasks = get_Link6_demo1_tasks_simple();
    
    Task example_task(manip);

    for (int i = 0; i < demo1_tasks.size(); i++) {
        example_task.add_joint_space_task(JointTask(demo1_tasks[i]));
    }

    OrderConstraint expected_constraint;
    expected_constraint.earlier = 0;
    expected_constraint.later = 1;
    example_task.add_order_constraint(expected_constraint.earlier, expected_constraint.later);
    CHECK(expected_constraint.earlier == example_task.order_constraints[0].earlier);
    CHECK(expected_constraint.later == example_task.order_constraints[0].later);

    expected_constraint.earlier = demo1_tasks.size() - 1;
    expected_constraint.later = demo1_tasks.size() - 2;
    example_task.add_order_constraint(expected_constraint.earlier, expected_constraint.later);
    CHECK(expected_constraint.earlier == example_task.order_constraints[1].earlier);
    CHECK(expected_constraint.later == example_task.order_constraints[1].later);

    // Throws error
    std::cout << "Should give 2 errors: " << std::endl;
    example_task.add_order_constraint(example_task.joint_space_tasks.size() + 1, 1);
    example_task.add_order_constraint(1, example_task.joint_space_tasks.size() + 1);
}

TEST_CASE("Task struct: add_domain_constraint() function test", "[Task]") {
    auto manip = get_generic_Link6();
    auto demo1_tasks = get_Link6_demo1_tasks_simple();
    
    Task example_task(manip);

    for (int i = 0; i < demo1_tasks.size(); i++) {
        example_task.add_joint_space_task(JointTask(demo1_tasks[i]));
    }
    
    example_task.start_position = get_Link6_home();

    DomainConstraint expected_constraint;
    expected_constraint.task_id = 0;
    expected_constraint.domain = Array(demo1_tasks.size());
    expected_constraint.domain[0] = 1;
    expected_constraint.domain[1] = 1;
    example_task.add_domain_constraint(expected_constraint.task_id, expected_constraint.domain);
    CHECK(expected_constraint.task_id == example_task.domain_constraints[0].task_id);
    CHECK(is_close(expected_constraint.domain, example_task.domain_constraints[0].domain));

    example_task.setup();

    CHECK(example_task.task_domain(0, 0) == 1);
    CHECK(example_task.task_domain(0, 1) == 1);
    CHECK(example_task.task_domain(0, 2) == 0);
    CHECK(example_task.task_domain(0, 3) == 0);
    CHECK(example_task.task_domain(0, 4) == 0);
    CHECK(example_task.task_domain(0, 5) == 0);
}

TEST_CASE("Task struct: setup() function test", "[Task]") {
    auto manip = get_generic_Link6();
    auto demo1_tasks = get_Link6_demo1_tasks_simple();
    
    Task example_task(manip);

    for (int i = 0; i < demo1_tasks.size(); i++) {
        example_task.add_joint_space_task(JointTask(demo1_tasks[i]));
    }

    example_task.start_position = get_Link6_home();

    example_task.setup();

    Matrix expected_cost(6, 6);
    expected_cost(0, 0) = 0.0000;
    expected_cost(0, 1) = 0.6926;
    expected_cost(0, 2) = 0.8050;
    expected_cost(0, 3) = 0.8501;
    expected_cost(0, 4) = 0.6979;
    expected_cost(0, 5) = 0.6814;
                            
    expected_cost(1, 0) = 0.7231;
    expected_cost(1, 1) = 0.0000;
    expected_cost(1, 2) = 0.7269;
    expected_cost(1, 3) = 0.7720;
    expected_cost(1, 4) = 0.6401;
    expected_cost(1, 5) = 0.6061;
    
    expected_cost(2, 0) = 0.7623;
    expected_cost(2, 1) = 0.6409;
    expected_cost(2, 2) = 0.0000;
    expected_cost(2, 3) = 0.8176;
    expected_cost(2, 4) = 0.6653;
    expected_cost(2, 5) = 0.6486;
    
    expected_cost(3, 0) = 0.6906;
    expected_cost(3, 1) = 0.6714;
    expected_cost(3, 2) = 0.7007;
    expected_cost(3, 3) = 0.0000;
    expected_cost(3, 4) = 0.5891;
    expected_cost(3, 5) = 0.5702;
    
    expected_cost(4, 0) = 0.7511;
    expected_cost(4, 1) = 0.6358;
    expected_cost(4, 2) = 0.7613;
    expected_cost(4, 3) = 0.8064;
    expected_cost(4, 4) = 0.0000;
    expected_cost(4, 5) = 0.6371;
    
    expected_cost(5, 0) = 0.6781;
    expected_cost(5, 1) = 0.5738;
    expected_cost(5, 2) = 0.6883;
    expected_cost(5, 3) = 0.7334;
    expected_cost(5, 4) = 0.5748;
    expected_cost(5, 5) = 0.0000;

    Array expected_cost_from_start = {0.5306,  0.4260,  0.5107,  0.3614,  0.6484,  0.4641};
    Array expected_minimum_cost_to_reach = {0.6781,  0.5738,  0.6883,  0.7334,  0.5748,  0.5702};
    Matrix expected_task_domain(6, 6);
    expected_task_domain(0, 0) = 1.0; 
    expected_task_domain(0, 1) = 1.0; 
    expected_task_domain(0, 2) = 1.0; 
    expected_task_domain(0, 3) = 1.0; 
    expected_task_domain(0, 4) = 1.0; 
    expected_task_domain(0, 5) = 1.0;
                            
    expected_task_domain(1, 0) = 1.0; 
    expected_task_domain(1, 1) = 1.0; 
    expected_task_domain(1, 2) = 1.0; 
    expected_task_domain(1, 3) = 1.0; 
    expected_task_domain(1, 4) = 1.0; 
    expected_task_domain(1, 5) = 1.0;
    
    expected_task_domain(2, 0) = 1.0; 
    expected_task_domain(2, 1) = 1.0; 
    expected_task_domain(2, 2) = 1.0; 
    expected_task_domain(2, 3) = 1.0; 
    expected_task_domain(2, 4) = 1.0; 
    expected_task_domain(2, 5) = 1.0;
    
    expected_task_domain(3, 0) = 1.0; 
    expected_task_domain(3, 1) = 1.0; 
    expected_task_domain(3, 2) = 1.0; 
    expected_task_domain(3, 3) = 1.0; 
    expected_task_domain(3, 4) = 1.0; 
    expected_task_domain(3, 5) = 1.0;
    
    expected_task_domain(4, 0) = 1.0; 
    expected_task_domain(4, 1) = 1.0; 
    expected_task_domain(4, 2) = 1.0; 
    expected_task_domain(4, 3) = 1.0; 
    expected_task_domain(4, 4) = 1.0; 
    expected_task_domain(4, 5) = 1.0;
    
    expected_task_domain(5, 0) = 1.0; 
    expected_task_domain(5, 1) = 1.0; 
    expected_task_domain(5, 2) = 1.0; 
    expected_task_domain(5, 3) = 1.0; 
    expected_task_domain(5, 4) = 1.0; 
    expected_task_domain(5, 5) = 1.0;

    CHECK(is_close(example_task.cost, expected_cost, 1e-4));
    CHECK(is_close(example_task.cost_from_start, expected_cost_from_start, 1e-4));
    CHECK(is_close(example_task.minimum_cost_to_reach, expected_minimum_cost_to_reach, 1e-4));
    CHECK(is_close(example_task.task_domain, expected_task_domain, 1e-4));
}
