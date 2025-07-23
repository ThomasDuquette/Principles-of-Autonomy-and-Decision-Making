#pragma once
#include "blast_rush.h"
#include "task.hpp"

struct CSP_Variable;

struct CSP_Domain {
    std::vector<int> domain;
};

struct CSP_Constraint {
    std::vector<CSP_Variable*> input_var;
    std::vector<CSP_Variable*> output_var;

    virtual bool consistent(std::vector<int> a, int x) = 0; // Pure virtual
    virtual ~CSP_Constraint() = default;
};

struct CSP_Variable {
    CSP_Domain domain;
    std::vector<CSP_Constraint*> from_arcs;
    std::vector<CSP_Constraint*> to_arcs;
};

struct CSP {
    std::vector<std::unique_ptr<CSP_Variable>> variables;
    std::vector<std::unique_ptr<CSP_Constraint>> constraints;
    std::vector<int> result;
;

    int select_value(CSP_Variable* current_variable) {
        while (current_variable->domain.domain.size() > 0) {
            auto x = current_variable->domain.domain[0];
            current_variable->domain.domain.erase(current_variable->domain.domain.begin());

            bool consistent = true;
            for (auto cons : current_variable->to_arcs) {
                if (!cons->consistent(result, x)) {
                    consistent = false;
                    break;
                }
            }
            if (consistent) {
                return x;
            }
        }
        return -1;
    }

    std::vector<int> backtrack() {
        result.resize(variables.size());
        
        std::vector<CSP_Domain> domains;
        domains.reserve(variables.size());
        
        for (int i = 0; i < variables.size(); i++) {
            domains.push_back(variables[i].get()->domain);
        }
        
        int i = 0;
        while (i >= 0 && i < variables.size()) {
            int current_value = select_value(variables[i].get());
            result[i] = current_value;
            if (current_value == -1) {
                for (int j = i; j < variables.size(); j++) {
                    variables[j].get()->domain = domains[j];
                }
                i = i-1;
            }
            else {
                i = i+1;
            }
        }
        if (i == -1) {
            return {};
        } else {
            return result;
        }
    }
    
    int select_value_fc(CSP_Variable* current_variable, int current_variable_idx) {
        std::vector<CSP_Domain> domains;
        domains.reserve(variables.size() - current_variable_idx);
        
        for (int i = current_variable_idx; i < variables.size(); i++) {
            domains.push_back(variables[i].get()->domain);
        }
        
        while (current_variable->domain.domain.size() > 0) {
            auto x = current_variable->domain.domain[0];
            current_variable->domain.domain.erase(current_variable->domain.domain.begin());
            
            // Variable is consistent with past choices
            bool consistent = true;
            for (auto cons : current_variable->to_arcs) {
                if (!cons->consistent(result, x)) {
                    consistent = false;
                    break;
                }
            }

            // Variable is consistent with future variables (no future variable has empty domain) -- Forward Checking
            auto current_result = result;
            if (consistent) {
                current_result[current_variable_idx] = x;

                for (auto cons : current_variable->from_arcs) {
                    consistent = true;
                    for (int v = 0; v < cons->output_var.size(); v++) {
                        // remove inconsistent values from domains
                        for (int d = cons->output_var[v]->domain.domain.size()-1; d >= 0; d--) {
                            if (!cons->consistent(current_result, cons->output_var[v]->domain.domain[d])) {
                                cons->output_var[v]->domain.domain.erase(cons->output_var[v]->domain.domain.begin() + d);
                            }
                        }
                        // No more domain : value x is rejected
                        if (cons->output_var[v]->domain.domain.size() == 0) {
                            consistent = false;
                            
                            // reset all domains
                            for (int i = current_variable_idx; i < cons->output_var.size(); i++) {
                                variables[i].get()->domain = domains[i];
                            }
                            break;
                        }
                    }
                    if (!consistent) {
                        break;
                    }
                }

                if (consistent) {
                    return x;
                }
            }
        }
        return -1;
    }

    std::vector<int> backtrack_fc() {
        result.resize(variables.size());
        
        std::vector<CSP_Domain> domains;
        domains.reserve(variables.size());
        
        for (int i = 0; i < variables.size(); i++) {
            domains.push_back(variables[i].get()->domain);
        }
        
        int i = 0;
        while (i >= 0 && i < variables.size()) {
            int current_value = select_value_fc(variables[i].get(), i);
            result[i] = current_value;
            if (current_value == -1) {
                for (int j = i; j < variables.size(); j++) {
                    variables[j].get()->domain = domains[j];
                }
                i = i-1;
            }
            else {
                i = i+1;
            }
        }
        if (i == -1) {
            return {};
        } else {
            return result;
        }
    }
};
