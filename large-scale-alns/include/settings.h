/*
Settings struct for the metaheuristic, stores and loads the settings file.

Written by: Siem ter Braake, 2024
*/

#ifndef SETTINGS_H
#define SETTINGS_H


#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>

void parseKeyValue(const std::string& line, std::unordered_map<std::string, std::string>& keyValueMap);

struct Settings { 
    // Param variables
    std::string d_instName;          // Name of the instance
    std::string d_distMetric;        // Distance metric to use, either euclidean or haversine
    float d_noiseLevel;              // Noise level for the noisy heuristics
    float d_alnsAlpha;               // Alpha parameter for the ALNS
    float d_tempStartSA;             // Initial temperature for simulated annealing
    float d_tempEndSA;               // End temperature for simulated annealing
    float d_excessiveRoutePenalty;   // Penalty for excessive route time

    std::size_t d_randomSeed;             // Random seed
    std::size_t d_nThreads;               // Number of threads
    std::size_t d_nConstructionTries;     // Number of tries for construction heuristic    
    std::size_t d_nDestroyOps;            // Number of destroy operations
    std::size_t d_nRepairOps;             // Number of repair operations
    std::size_t d_nNeighborConsider;      // Number of neighbors to consider for the local search
    std::size_t d_maxNeighborhoodSize;    // Maximum size of the neighborhood
    std::size_t d_subproblemSize;         // Size of the subproblem for LBO
    std::size_t d_nIterations;            // Number of iterations for the Metaheuristic
    std::size_t d_nCheckFeasible;         // Number of iterations before checking feasibility
    std::size_t d_nThreadLoops;           // Number of loops for the parallel executor
    std::size_t d_sortType;               // Type of node sorting before construction heuristic
    
    bool d_verbose;             // Whether to print verbose output
    bool d_local;               // Whether to run the local search
    bool d_restricedServiceArea;// Whether to route based on the territories defined by tall
    bool d_writeFiles;          // Whether to write the solution and logs to file
    bool d_simulatedAnnealing;  // Whether to use simulated annealing
    
    // Default parameters
    Settings();
    bool load(const std::string path_settings);
};

#endif // SETTINGS_H