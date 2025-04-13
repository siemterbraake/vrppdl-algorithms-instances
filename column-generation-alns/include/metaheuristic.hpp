/*
Metaheuristic functions and data structures, performs the metaheuristic algorithm and generates a solution.

Written by: Siem ter Braake, 2024
*/

#ifndef METAHEURISTIC_H
#define METAHEURISTIC_H

#include "instance.hpp"
#include "graph.hpp"
#include "settings.hpp"
#include "alnsSolution.hpp"
#include "duals.hpp"
#include "label.hpp"
#include "capacitatedSortedDeque.hpp"

#include <random>
#include <cassert>
#include <limits>
#include <vector>
#include <tuple>
#include <cmath>
#include <numeric>
#include <filesystem>
#include <chrono>
#include <iostream>
#include <thread>
#include <iomanip>
#include <deque>
#include <algorithm>

struct Metaheuristic {
    const Settings& d_settings; // Settings data
    const Instance& d_inst;     // Instance data
    const Graph& d_graph;       // Graph data
    ALNSSolution d_tempSol;     // Temporary solution optimized by the current iteration
    ALNSSolution d_curSol;      // Current solution stored when the temporary solution is accepted
    ALNSSolution d_bestSol;     // Best solution found so far
    Duals d_duals;              // Dual variables for the current run

    std::set<std::size_t> d_solCache; // Hashes of the solutions found

    std::vector<double> d_tempObjVals;                // Objective values of temporary solution per iteration
    std::vector<double> d_curObjVals;                 // Objective values of current solution per iteration
    std::vector<double> d_bestObjVals;                // Objective values of best solution per iteration
    std::vector<double> d_wDestroyOp;                 // Weights of destroy operations
    std::vector<double> d_wRepairOp;                  // Weights of repair operations
    std::vector<double> d_pThresholds;                // Temperatures for simulated annealing
    CapacitatedSortedDeque<Label>  d_labels;          // Labels for the negative reduced cost routes
    CapacitatedSortedDeque<ALNSSolution> d_bestSolutions; // Best solutions found so far
    std::mt19937 d_gen;                               // Random number generator

    bool d_initialize;                 // Flag to indicate if initialize mode is active
    unsigned int d_nIterations;        // Number of iterations
    unsigned int d_nUnimproved;        // Number of unimproved iterations
    std::string d_idRun;               // ID of the current run
    double d_bestObjVal;               // Best objective value
    double d_pThreshold;               // Current threshold for pta
    double d_pThresholdMax;            // Maximum threshold for pta
    unsigned short d_sizeNeighborhood; // Size of the neighborhood of the current iteration
    unsigned short d_iRepairOp;        // Index of current repair operation
    unsigned short d_iDestroyOp;       // Index of current destroy operation
    unsigned short d_curScore;         // Current score of the operator
    unsigned short d_parallelIter;     // Iteration of the parallel run

    // Core Functions -> metaheuristic.cpp
    Metaheuristic(const Settings &settings, const Instance &inst, const Graph &graph, unsigned short seed);
    void setStandardParams();
    void run();
    void setNeighorhoodSize();
    void findInitialSolution();
    void setOperationIdx();
    void performDestroy();
    void performRepair();
    void evaluateSolution(unsigned int i);
    void resetPTA(unsigned int i);   
    void updateOpWeights();
    void setDuals(Duals duals);
    void storeLabel(unsigned short idDepot, unsigned short idVehicleType, unsigned short idRoute, unsigned short idNode, unsigned short idSL, unsigned short pos, double reducedCost);
    std::deque<Label> getLabels();
    void clear();
    void updateProgressBar(unsigned int i);
    void writeLogs(double timeElapsed, bool error = false) const;
    void writeSummary(double timeElapsed, std::string path) const;
    void writeSolutionTrend(std::string path) const;
    void writeThresholds(std::string path) const;

    // Destroy functions -> metaheuristicDestroy.cpp
    void destroyDynamic(unsigned short idAnchor, double routeCoef); // #0 dynamic destroy
    void destroyRandomRoute(); // #1 Randomly remove a route from the solution
    void destroyGreedyRoute(bool noise); // #2 Remove the worst route from the solution with noise (without noise not used)
    void destroyWaiting(); // #4 Remove the nodes that cause waiting time
    void destroyRandom(); // NOT USED Randomly remove a customer node from the solution
    void destroyGreedy(bool noise); // NOT USED Remove the worst customer node from the solution

    // Repair functions -> metaheuristicRepair.cpp
    void repairGreedy(bool noise); // #0 Greedily insert a customer node into the solution (#1 with noise)
    void repairGreedyLocal(bool noise); // #0 Greedily insert a customer node into the solution using the local search (#1 with noise)
    void repairRandomFirstFeasible(); // #2 Insert a customer node into the solution using the first feasible insertion
    bool insertNode(const Node &req, unsigned short pos, unsigned short &idDepot, 
                    unsigned short &idVehicleType, unsigned short &idRoute, unsigned short &idSL,
                    int twDepotStart, int twDepotEnd, double costInsert); // Insert a node into the solution
    
    // Metaheuristic operator cannot be moved
    Metaheuristic& operator=(const Metaheuristic&) = delete;

};

#endif // METAHEURISTIC_H