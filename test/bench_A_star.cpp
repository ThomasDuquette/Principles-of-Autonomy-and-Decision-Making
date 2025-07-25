#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_ENABLE_BENCHMARKING

#include "catch2/catch.hpp"
#include "blast_rush.h"
#include "../extern/blast_rush/tests/test_helper/test_helper.hpp"
#include "../extern/blast_rush/tests/test_helper/test_functions.hpp"
#include "../utilities.hpp"
#include "../A_star.hpp"

TEST_CASE("test A_star() function with no constraints", "[A_star]") {
    auto manip = get_generic_Link6();
    World world;
    auto demo1_tasks = get_Link6_demo1_tasks_simple();
    
    TaskSequencingProblem example_task(manip);

    for (int i = 0; i < demo1_tasks.size(); i++) {
        example_task.add_task(Task(demo1_tasks[i]));
    }

    example_task.start_position = get_Link6_home();
    BENCHMARK("setup()") {
        example_task.setup(world);
    };

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
    World world;
    auto demo1_tasks = get_Link6_demo1_tasks_simple();
    
    TaskSequencingProblem example_task(manip);

    for (int i = 0; i < demo1_tasks.size(); i++) {
        example_task.add_task(Task(demo1_tasks[i]));
    }

    example_task.start_position = get_Link6_home();

    example_task.add_order_constraint(0, 1);

    example_task.setup(world);

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
    World world;
    auto demo1_tasks = get_Link6_demo1_tasks_simple();
    
    TaskSequencingProblem example_task(manip);

    for (int i = 0; i < demo1_tasks.size(); i++) {
        example_task.add_task(Task(demo1_tasks[i]));
    }

    example_task.start_position = get_Link6_home();

    Array domain(demo1_tasks.size());
    domain[0] = 1.0;
    domain[1] = 1.0;

    example_task.add_domain_constraint(0, domain);

    example_task.setup(world);

    bool success = false;
    Array solution;
    BENCHMARK("A_star() with domain constraints") {
        solution = A_star(example_task, &success);
    };


    Array expected_solution = {3.0, 0.0, 5.0, 4.0, 1.0, 2.0};

    CHECK(success);
    CHECK(is_close(expected_solution, solution));
}

// WIth cartesian 

TEST_CASE("test A_star() function with cartesian with no constraints", "[A_star]") {
    auto manip = get_generic_Link6();
    World world;
    auto demo1_tasks = get_Link6_demo1_tasks_simple();
    
    TaskSequencingProblem example_task(manip);

    for (int i = 0; i < demo1_tasks.size(); i++) {
        example_task.add_task(Task(demo1_tasks[i]));
    }
    CartesianTask task1;
    example_task.add_task(Task(task1));

    example_task.start_position = get_Link6_home();
    example_task.setup(world);

    bool success = false;
    std::vector<std::vector<Array>> joint_space_solution;
    Array solution = {};
    BENCHMARK("A* with cartesian and no constraints") {
        solution = A_star(example_task, &success, &joint_space_solution);
    };

    Array expected_solution = {3.0, 5.0, 4.0, 1.0, 2.0, 0.0, 6.0, 15.0};

    CHECK(success);
    CHECK(is_close(expected_solution, solution));
}

TEST_CASE("test A_star() function with cartesian with order constraints", "[A_star]") {
    auto manip = get_generic_Link6();
    World world;
    auto demo1_tasks = get_Link6_demo1_tasks_simple();
    
    TaskSequencingProblem example_task(manip);

    for (int i = 0; i < demo1_tasks.size(); i++) {
        example_task.add_task(Task(demo1_tasks[i]));
    }
    CartesianTask task1;
    example_task.add_task(Task(task1));

    example_task.start_position = get_Link6_home();

    example_task.add_order_constraint(0, 1);

    example_task.setup(world);

    bool success = false;
    std::vector<std::vector<Array>> joint_space_solution;
    Array solution = {};
    BENCHMARK("A* with cartesian and order constraints") {
        solution = A_star(example_task, &success, &joint_space_solution);
    };

    Array expected_solution = {3.0, 5.0, 4.0, 0.0, 2.0, 1.0, 6.0, 15.0};

    CHECK(success);
    CHECK(is_close(expected_solution, solution));
}

TEST_CASE("test A_star() function with cartesian with domain constraints", "[A_star]") {
    auto manip = get_generic_Link6();
    World world;
    auto demo1_tasks = get_Link6_demo1_tasks_simple();
    
    TaskSequencingProblem example_task(manip);

    for (int i = 0; i < demo1_tasks.size(); i++) {
        example_task.add_task(Task(demo1_tasks[i]));
    }
    CartesianTask task1;
    example_task.add_task(Task(task1));

    example_task.start_position = get_Link6_home();

    Array domain(demo1_tasks.size());
    domain[0] = 1.0;
    domain[1] = 1.0;

    example_task.add_domain_constraint(0, domain);

    example_task.setup(world);

    bool success = false;
    Array solution = {};
    BENCHMARK("A* with cartesian and domain constraints") {
        solution = A_star(example_task, &success);
    };

    Array expected_solution = {3.0, 0.0, 5.0, 4.0, 2.0, 1.0, 6.0, 15.0};

    CHECK(success);
    CHECK(is_close(expected_solution, solution));
}