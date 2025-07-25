#define CATCH_CONFIG_MAIN
#include "catch2/catch.hpp"
#include "blast_rush.h"
#include "../extern/blast_rush/tests/test_helper/test_helper.hpp"
#include "../extern/blast_rush/tests/test_helper/test_functions.hpp"
#include "../utilities.hpp"
#include "../A_star.hpp"

TEST_CASE("test extract_solution() function", "[A_star]") {
    Node n1;
    Node n2;
    Node n3;
    Node n4;

    n1.n_affected_tasks = 4;
    n1.id = 1;
    n1.parent = &n2;

    n2.parent = &n4;
    n2.id = 2;

    n4.parent = &n3;
    n4.id = 4;
    
    n3.id = 3;

    auto solution = extract_solution(n1);
    Array expected_solution = {3, 4, 2, 1};

    CHECK(is_close(solution, expected_solution));
}

TEST_CASE("test insert_node() function", "[A_star]") {
    std::list<Node> node_list;

    Node n1;
    n1.id = 1.0;
    n1.total_cost = 1.0;
    Node n2;
    n2.id = 2.0;
    n2.total_cost = 2.0;
    Node n3;
    n3.id = 3.0;
    n3.total_cost = 3.0;
    Node n4;
    n4.id = 4.0;
    n4.total_cost = 4.0;

    insert_node(node_list, n2);
    insert_node(node_list, n1);
    insert_node(node_list, n4);
    insert_node(node_list, n3);

    Array expected_solution = {1.0, 2.0, 3.0, 4.0};
    Array solution(4);
    auto it = node_list.begin();
    for (int i = 0; i < 4 && it != node_list.end(); ++i, ++it) {
        solution[i] = (real)it->id;
    }

    CHECK(is_close(solution, expected_solution));
}

TEST_CASE("test is_consistent() function", "[A_star]") {
    auto manip = get_generic_Link6();
    World world;
    auto demo1_tasks = get_Link6_demo1_tasks_simple();

    Node n0;
    n0.id = 0;
    n0.n_affected_tasks = 2;

    Node n1;
    n1.id = 1;
    n1.parent = &n0;
    n1.n_affected_tasks = 2;
    
    TaskSequencingProblem example_task(manip);

    for (int i = 0; i < demo1_tasks.size(); i++) {
        example_task.add_task(Task(demo1_tasks[i]));
    }

    example_task.start_position = get_Link6_home();
    
    auto example_task_consistent = example_task;
    auto example_task_inconsistent = example_task;

    example_task_consistent.add_order_constraint(0, 1);
    example_task_consistent.setup(world);

    CHECK(is_consistent(example_task_consistent, n1));
    
    example_task_inconsistent.add_order_constraint(1, 0);
    example_task_inconsistent.setup(world);
    
    CHECK(!is_consistent(example_task_inconsistent, n1));
}

TEST_CASE("test A_star() function with no constraints", "[A_star]") {
    auto manip = get_generic_Link6();
    World world;
    auto demo1_tasks = get_Link6_demo1_tasks_simple();
    
    TaskSequencingProblem example_task(manip);

    for (int i = 0; i < demo1_tasks.size(); i++) {
        example_task.add_task(Task(demo1_tasks[i]));
    }

    example_task.start_position = get_Link6_home();
    example_task.setup(world);

    bool success = false;
    auto solution = A_star(example_task, &success);

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
    auto solution = A_star(example_task, &success);

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
    auto solution = A_star(example_task, &success);

    Array expected_solution = {3.0, 0.0, 5.0, 4.0, 1.0, 2.0};

    CHECK(success);
    CHECK(is_close(expected_solution, solution));
}

// With cartesian

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
    auto solution = A_star(example_task, &success, &joint_space_solution);

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
    auto solution = A_star(example_task, &success, &joint_space_solution);

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
    auto solution = A_star(example_task, &success);

    Array expected_solution = {3.0, 0.0, 5.0, 4.0, 2.0, 1.0, 6.0, 15.0};

    CHECK(success);
    CHECK(is_close(expected_solution, solution));
}
