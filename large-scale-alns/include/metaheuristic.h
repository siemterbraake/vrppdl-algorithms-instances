/*
Metaheuristic functions and data structures, performs the metaheuristic algorithm and generates a solution.

Written by: Siem ter Braake, 2024
*/

#ifndef METAHEURISTIC_H
#define METAHEURISTIC_H

#include "loader.h"
#include "graph.h"
#include "settings.h"
#include "solution.h"

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

struct Metaheuristic {
    Graph d_graph;       // Graph data
    Instance d_inst;     // Instance data
    Settings d_settings; // Settings data
    Solution d_tempSol;  // Temporary solution optimized by the current iteration
    Solution d_curSol;   // Current solution stored when the temporary solution is accepted
    Solution d_bestSol;  // Best solution found so far

    std::set<std::size_t> d_solCache; // Hashes of the solutions found

    std::vector<float> d_tempObjVals; // Objective values of temporary solution per iteration
    std::vector<float> d_curObjVals;  // Objective values of current solution per iteration
    std::vector<float> d_bestObjVals; // Objective values of best solution per iteration
    std::vector<float> d_wDestroyOp;  // Weights of the operators
    std::vector<float> d_wRepairOp;   // Weights of the operators
    std::vector<float> d_pThresholds; // Thresholds for PTA
    std::mt19937 d_gen;               // Random number generator

    std::string d_idRun;            // ID of the current run
    std::string d_pathOutput;       // Path to the output folder
    float d_bestObjVal;             // Best objective value
    float d_pThreshold;             // Threshold percentage for PTA acceptance
    float d_pThresholdMax;          // Max threshold percentage for PTA at current stage of algo
    std::size_t d_sizeNeighborhood; // Size of the neighborhood of the current iteration
    std::size_t d_iRepairOp;        // Index of current repair operation
    std::size_t d_iDestroyOp;       // Index of current destroy operation
    std::size_t d_curScore;         // Current score of the operator
    std::size_t d_nUnimproved;      // Number of unimproved iterations
    std::size_t d_parallelIter;     // Iteration of the parallel run
    std::size_t d_nIterRestart;     // Number of iterations before restarting the simulated annealing

    // Core Functions -> metaheuristic.cpp
    Metaheuristic(std::string idRun);
    Metaheuristic(Settings &settings, const Instance &inst, const Graph &graph, std::string pathOutput, std::string &idRun, std::size_t seed);
    void setStandardParams();
    void run();
    float runSubproblem(std::size_t idSubproblem);
    float calcSubproblemCost(std::size_t idSubproblem) const;
    void setNeighorhoodSize();
    void findInitialSolution();
    void setOperationIdx();
    void performDestroy();
    void performRepair();
    void evaluateSolution(std::size_t i);
    void resetPTA(std::size_t i); // Resets the parameters for PTA   
    void updateOpWeights();
    const std::vector<std::vector<std::size_t>>& getSubproblems() const;
    std::size_t getSubproblemCount() const;
    std::vector<std::vector<float>> getSubproblemEmbeddings(std::size_t idSubproblem) const;
    void updateSubproblems();
    void updateProgressBar(std::size_t i);
    void writeLogs(float timeElapsed, bool error = false, bool python = false) const;
    void writeSummary(float timeElapsed, std::string path) const;
    void writeSolutionTrend(std::string path) const;
    void writeThresholds(std::string path) const;
    Solution getBestSolution() const;
    void setBestSolution(Solution sol);

    // Destroy functions -> metaheuristicDestroy.cpp
    void destroyDynamic(std::size_t idAnchor, float routeCoef); // #0 dynamic destroy
    void destroyRandomRoute(); // #1 Randomly remove a route from the solution
    void destroyGreedyRoute(bool noise); // #2 Remove the worst route from the solution with noise (without noise not used)
    void destroyExcessiveRoute(); // #3 Remove the route with the highest excessive route penalty
    void destroyWaiting(); // #4 Remove the nodes that cause waiting time
    void destroyRandom(); // NOT USED Randomly remove a customer node from the solution
    void destroyGreedy(bool noise); // NOT USED Remove the worst customer node from the solution

    // Repair functions -> metaheuristicRepair.cpp
    void repairGreedy(bool noise); // #0 Greedily insert a customer node into the solution (#1 with noise)
    void repairGreedyLocal(bool noise, bool restrictServiceArea); // #0 Greedily insert a customer node into the solution using the local search (#1 with noise)
    void repairRandomFirstFeasible(bool restrictServiceArea); // #2 Insert a customer node into the solution using the first feasible insertion

    // Patching functions -> metaheuristicPatching.cpp
    void patchInfeasibleRequests();
};

#endif // HEUR_H