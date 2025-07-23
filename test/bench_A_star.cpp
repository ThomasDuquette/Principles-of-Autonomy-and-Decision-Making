#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_ENABLE_BENCHMARKING

#include "catch2/catch.hpp"
#include "blast_rush.h"
#include "../extern/blast_rush/tests/test_helper/test_helper.hpp"
#include "../extern/blast_rush/tests/test_helper/test_functions.hpp"
#include "../A_star.hpp"
#include "../utilities.hpp"

TEST_CASE("test A_star() function with no constraints", "[A_star]") {
    auto manip = get_generic_Link6();
    auto demo1_tasks = get_Link6_demo1_tasks_simple();
    
    Task example_task(manip);

    for (int i = 0; i < demo1_tasks.size(); i++) {
        example_task.add_joint_space_task(JointTask(demo1_tasks[i]));
    }

    example_task.start_position = get_Link6_home();
    example_task.setup();

    bool success = false;
    Array solution;
    BENCHMARK("A_star() with no constraints") {
        solution = A_star(example_task, &success);
    };

    Array expected_solution = {3.0, 5.0, 4.0, 1.0, 2.0, 0.0};

    CHECK(success);
    CHECK(is_close(expected_solution, solution));
}

TEST_CASE("test A_star() function with order constraints", "[A_star]") {
    auto manip = get_generic_Link6();
    auto demo1_tasks = get_Link6_demo1_tasks_simple();
    
    Task example_task(manip);

    for (int i = 0; i < demo1_tasks.size(); i++) {
        example_task.add_joint_space_task(JointTask(demo1_tasks[i]));
    }

    example_task.start_position = get_Link6_home();

    example_task.add_order_constraint(0, 1);

    example_task.setup();

    bool success = false;
    Array solution;
    BENCHMARK("A_star() with order constraints") {
        solution = A_star(example_task, &success);
    };


    Array expected_solution = {3.0, 5.0, 0.0, 4.0, 1.0, 2.0};

    CHECK(success);
    CHECK(is_close(expected_solution, solution));
}

TEST_CASE("test A_star() function with domain constraints", "[A_star]") {
    auto manip = get_generic_Link6();
    auto demo1_tasks = get_Link6_demo1_tasks_simple();
    
    Task example_task(manip);

    for (int i = 0; i < demo1_tasks.size(); i++) {
        example_task.add_joint_space_task(JointTask(demo1_tasks[i]));
    }

    example_task.start_position = get_Link6_home();

    Array domain(demo1_tasks.size());
    domain[0] = 1.0;
    domain[1] = 1.0;

    example_task.add_domain_constraint(0, domain);

    example_task.setup();

    bool success = false;
    Array solution;
    BENCHMARK("A_star() with domain constraints") {
        solution = A_star(example_task, &success);
    };


    Array expected_solution = {3.0, 0.0, 5.0, 4.0, 1.0, 2.0};

    CHECK(success);
    CHECK(is_close(expected_solution, solution));
}