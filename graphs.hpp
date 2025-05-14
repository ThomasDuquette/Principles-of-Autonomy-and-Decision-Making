#include "blast_rush.h"

using namespace blast;

bool is_in_list(State state, std::vector<State> list);
bool is_same_state(State state1, State state2);
template <typename T>
std::vector<T> flip(std::vector<T> v);

enum uninformed_search_type {
    DEPTH_FIRST,
    BREADTH_FIRST
};

struct State {
    int idx;

    std::vector<State> get_reachable_states() {
        switch (idx) {
            case 1: 

                break;
            default:
                std::cerr << "Error : idx is not valid" << std::endl;
                break;
        }
    }
};

struct Node {
    Node* parent;
    State state;
    float path_cost = 0;
};

struct UninformedSearch {
    std::vector<Node> search_queue;
    std::vector<State> visited_states;
    Matrix costs; // cost to go from <row> to <col>

    uninformed_search_type search_type = uninformed_search_type::DEPTH_FIRST;
    bool use_visited_list = true;

    void initialize_search(Node start_node) {
        search_queue.clear();
        search_queue.resize(1);
        search_queue[0] = start_node;

        visited_states.clear();
        visited_states.resize(1);
        visited_states[0] = start_node.state;
    }

    std::vector<Node> get_extended_paths(Node node) {
        auto possible_states = node.state.get_reachable_states();
        std::vector<Node> result;
        result.reserve(possible_states.size());
        for (auto state : possible_states) {
            if (!use_visited_list || !is_in_list(state, visited_states)) {
                Node new_node;
                new_node.parent = &node;
                new_node.path_cost = node.path_cost + costs(node.state.idx, state.idx);
                new_node.state = state;
                result.push_back(new_node);
            }
        }
        return result;
    }

    void depth_first(State end) {
        bool found_path = false;
        while (!found_path) {
            if (is_same_state(search_queue[search_queue.size()-1].state, end)) {
                found_path = true;
                break;
            }
            auto next_nodes = get_extended_paths(search_queue[search_queue.size()-1]);

            next_nodes = flip(next_nodes);

            for (auto node : next_nodes) {
                search_queue.push_back(node);
                visited_states.push_back(node.state);
            }
        }
    }

    void breadth_first(State end) {
        bool found_path = false;
        while (!found_path) {
            auto next_nodes = get_extended_paths(search_queue[0]);

            for (auto node : next_nodes) {
                search_queue.push_back(node);
                visited_states.push_back(node.state);

                if (is_same_state(node.state, end)) {
                    found_path = true;
                    break;
                }
            }
        }
    }

    std::vector<State> search(State start, State end) {
        Node start_node;
        start_node.path_cost = 0;
        start_node.state = start;

        initialize_search(start_node);

        switch (search_type) {
            case uninformed_search_type::DEPTH_FIRST:
                depth_first(end);
                break;
            case uninformed_search_type::BREADTH_FIRST:
                breadth_first(end);
                break;
            default:
                std::cerr << "Error : Wrong search type" << std::endl;
                break;
        }

        std::vector<State> result;
        auto current_node = search_queue.back();
        while (true) {
            if (is_same_state(current_node.state, start)) {
                result.push_back(start);
                break;
            } else {
                result.push_back(current_node.state);
                current_node = *current_node.parent;
            }
        }

        return flip(result);
    }
};

bool is_in_list(State state, std::vector<State> list) {
    for (auto element : list) {
        if (state.idx == element.idx) {
            return true;
        }
    }
    return false;
}

bool is_same_state(State state1, State state2) {
    if (state1.idx != state2.idx) {
        return false;
    }
    return true;
}

template <typename T>
std::vector<T> flip(std::vector<T> v) {
    std::vector<T> results;
    results.reserve(v.size());
    for (int i = v.size() - 1; i >= 0; v++) {
        results.push_back(v[i]);
    }
}