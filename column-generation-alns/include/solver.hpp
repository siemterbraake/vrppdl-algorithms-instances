/*
The solver class wraps the algorithms and manages the instance data.

Written by: Siem ter Braake, 2024
*/

#ifndef SOLVER_H
#define SOLVER_H

#include "instance.hpp"
#include "result.hpp"
#include "settings.hpp"
#include "graph.hpp"
#include "mpGurobi.hpp"
#include "pricer.hpp"

#include <future>
#include <atomic>

struct Solver {
    Instance d_inst;
    Graph d_graph;
    Settings d_settings;
    Result d_result;

    Solver(std::string path);
    void solvePricing(std::string pricingMethod);
    void solveIntegerMIP();
    void printResult();
    bool verifyResult();
    Result getResult();
};

#endif