/*
The gurobi master problem class is responsible for solving the master problem in the column generation algorithm.

Written by: Casper Bazelmans, 2024
*/

#ifndef MASTER_PROBLEM_H
#define MASTER_PROBLEM_H

#include "instance.hpp"
#include "lpSolution.hpp"
#include "label.hpp"
#include "pricer.hpp"
#include "graph.hpp"
#include "metaheuristic.hpp"
#include "settings.hpp"
#include "result.hpp"

#include "gurobi_c++.h"

#include <vector>
#include <deque>
#include <set>
#include <tuple>
#include <iomanip>
#include <algorithm>
#include <cassert>
#include <chrono>


struct Variable
{
    double d_coefObjective;
    std::vector<double> d_coefsConstaint;

    Variable() : d_coefObjective(0.0) {}

    Variable(double coefObjective, std::vector<double> coefsConstraint): 
        d_coefObjective(coefObjective), d_coefsConstaint(coefsConstraint) {}
};



struct MasterProblemGRB
{
    GRBEnv d_env;
    LPSolution d_solRelaxed;
    LPSolution d_solInteger;
    Settings& d_settings;
    const Instance& d_inst;
    const Graph& d_graph;

    std::vector<Variable> d_xVariables;
    std::vector<Label> d_routes;
    std::vector<int> d_routeVehTypes;
    std::set<std::tuple<unsigned short, std::vector<unsigned short>, std::vector<unsigned char>>> d_routesInMP;

    unsigned short d_nVariables;
    unsigned short d_nCustConstraints;
    unsigned short d_nVehConstraints;
    unsigned short d_nSLConstraints;

    MasterProblemGRB(Settings& settings, const Instance& instance, const Graph& graph);
    void clear();
    void initDummyVar();
    void initALNS();
    void labelExact(Result& result, std::atomic<bool>& stopFlag);
    void labelALNS(Result& result, std::atomic<bool>& stopFlag);
    void printSolution();
    void writeSolution();
    void solveCurrentLP(bool relaxed);
    bool addPricedVariables(const std::deque<Label>& paths);
    double countCustomersPerRoute();
    void validateReducedCost(const std::deque<Label>& paths);
    void validateRealCost(double costReal, const Label &route);
    void validateReducedCostOfVariables();

    // Delete copy constructor and assignment operator since GRBEnv cannot be copied
    MasterProblemGRB& operator=(const MasterProblemGRB&) = delete;
};  


#endif // MASTER_PROBLEM_H