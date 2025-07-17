// #define CATCH_CONFIG_MAIN
// #define CATCH_CONFIG_ENABLE_BENCHMARKING
// #include "catch2/catch.hpp"
#include "../constraints_satisfaction_problem.hpp"
#include "../utilities.hpp"
#include <iostream>
#include <vector>

struct NQueensDomain : CSP_Domain {
    NQueensDomain(int n_queens) {
        domain.resize(n_queens);
        for (int i = 0; i < domain.size(); i++) {
            domain[i] = i;
        }
    }
};

struct NQueensVariable : CSP_Variable {
    int col;
    NQueensVariable(int pos, int n_queens) :
        col(pos)
     {
        domain = NQueensDomain(n_queens);
     }
};

struct NQueensConstraint : CSP_Constraint {
    NQueensConstraint(NQueensVariable& var_1, NQueensVariable& var_2) {
        input_var = {&var_1};
        output_var = {&var_2};
        var_1.from_arcs.push_back(this);
        var_2.to_arcs.push_back(this);
    }
    
    bool func(int col_1, int row_1, int col_2, int row_2) {
        // Attacks horizontally
        if (row_1 == row_2) {
            return false;
        }
        // Attacks diagonally
        if (row_2 - row_1 == col_2 - col_1) {
            return false;
        }
        if (row_1 - row_2 == col_2 - col_1) {
            return false;
        }
        // Does not attack
        return true;
    }

    bool consistent(std::vector<int> a, int x) override {
        auto* var1 = static_cast<NQueensVariable*>(input_var[0]);
        auto* var2 = static_cast<NQueensVariable*>(output_var[0]);

        if (func(var1->col, a[var1->col], var2->col, x)) {
            return true;
        }
        return false;
    }
};

struct NQueens : CSP {
    NQueens(int n_queens) {
        variables.resize(n_queens);
        for (int i = 0; i < n_queens; i++) {
            variables[i] = std::make_unique<NQueensVariable>(i, n_queens);
        }
    }
};

int main() {
    // --- RESULTS ---

    // PROBLEM 1 (Backtrack)
    // --- 5 x 5 ---
    // [ 0, 2, 4, 1, 3 ]
    // total time : 3 us
    // --- 10 x 10 ---
    // [ 0, 2, 5, 7, 9, 4, 8, 1, 3, 6 ]
    // total time : 228 us
    // --- 15 x 15 ---
    // [ 0, 2, 4, 1, 9, 11, 13, 3, 12, 8, 5, 14, 6, 10, 7 ]
    // total time : 7392 us
    // The time increases exponentially, as the problem difficulty.

    // PROBLEM 2 (Backtrack-Forward Check)
    // --- 5 x 5 ---
    // [ 0, 2, 4, 1, 3 ]
    // total time : 7 us
    // --- 10 x 10 ---
    // [ 0, 2, 5, 7, 9, 4, 8, 1, 3, 6 ]
    // total time : 498 us
    // --- 20 x 20 ---
    // [ 0, 2, 4, 1, 3, 12, 14, 11, 17, 19, 16, 8, 15, 18, 7, 9, 6, 13, 5, 10 ]
    // total time : 3.722653 s
    // 30 x 30
    // [ 0, 2, 4, 1, 3, 8, 10, 12, 14, 6, 22, 25, 27, 24, 21, 23, 29, 26, 28, 15, 11, 9, 7, 5, 17, 19, 16, 13, 20, 18 ]
    // total time : 39 m 43.262083 s  {0.0}


    // PROBLEM 3 
    // Part A is only running a provided (already compiled) code, so I'll skip it.
    // Part B asks us to give a change to the spare tire problem that could susbtantially complicate
    // it. I suggest the addition of a lock nut which requires a trip to the hardware store to get
    // the correct key, since you don't have it on you. This is more complex as it is an additional
    // task, but since you don't have your car, you would need to take a taxi. Thus you can call the 
    // taxi and do the other tasks while waiting, saving you some time.

    int n_queens = 30;

    // // --- PROBLEM 1 ---
    // // creating problem
    // NQueens problem_1(n_queens);
    
    // // adding constraints
    // for (int i = 0; i < n_queens-1; i++) {
    //     for (int j = i+1; j < n_queens; j++) {
    //         auto var1 = static_cast<NQueensVariable*>(problem_1.variables[i].get());
    //         auto var2 = static_cast<NQueensVariable*>(problem_1.variables[j].get());
    //         problem_1.constraints.push_back(std::make_unique<NQueensConstraint>(*var1, *var2));
    //     }
    // }
    
    // auto T1 = blast::get_tick_us();
    // auto solution = problem_1.backtrack();
    // auto T2 = blast::get_tick_us();

    // print(solution);
    // std::cout << "total time : " << (T2-T1) << " us" << std::endl;

    // --- PROBLEM 2 ---
    // creating problem
    NQueens problem_2(n_queens);
    
    // adding constraints
    for (int i = 0; i < n_queens-1; i++) {
        for (int j = i+1; j < n_queens; j++) {
            auto var1 = static_cast<NQueensVariable*>(problem_2.variables[i].get());
            auto var2 = static_cast<NQueensVariable*>(problem_2.variables[j].get());
            problem_2.constraints.push_back(std::make_unique<NQueensConstraint>(*var1, *var2));
        }
    }
    auto T1 = blast::get_tick_us();
    auto solution = problem_2.backtrack_fc();
    auto T2 = blast::get_tick_us();

    print(solution);
    std::cout << "total time : " << (T2-T1) << " us" << std::endl;


}
