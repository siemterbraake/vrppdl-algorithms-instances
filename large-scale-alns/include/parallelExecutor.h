/*
Parallel executor that can be used to run multiple instances of a given algorithm in parallel.

Written by: Siem ter Braake, 2024
*/

#ifndef PARALLELEXECUTOR_H
#define PARALLELEXECUTOR_H

#include "metaheuristic.h"
#include "settings.h"
#include "loader.h"
#include "graph.h"

#include <chrono>
#include <thread>

struct ParallelExecutor {
    Graph d_graph;                               // Graph data
    std::vector<Metaheuristic> d_metaheuristics; // Metaheuristic objects that are run by the threads
    Settings d_settings;                         // Settings data
    Instance d_inst;                             // Instance data
    std::vector<std::thread> d_threads;          // Threads that run the metaheuristic algorithm
    float d_bestObjVal;                          // Best objective
    std::size_t d_bestThread;                    // Best thread index of the current iteration

    // Core Functions -> parallelExecutor.cpp
    ParallelExecutor(std::string idRun);
    void run();
    void runSubproblem(std::size_t subproblems);
    void distributeBestSolution();
    const std::vector<std::vector<std::size_t>>& getSubproblems() const;
    float calcSubproblemCost(std::size_t idSubproblem) const;
    std::vector<std::vector<float>> getSubproblemEmbeddings(std::size_t idSubproblem) const;
    void writeLogs(float timeElapsed, bool error = false, bool python = false) const;

};

#endif