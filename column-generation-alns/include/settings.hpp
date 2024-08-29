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
    std::string d_distMetric;        // Distance metric to use, either euclidean or haversine
    
    double d_noiseLevel;             // Noise level for the noisy heuristics
    double d_alnsAlpha;              // Alpha parameter for the ALNS
    double d_tempStartSA;     // Initial temperature for simulated annealing
    double d_tempEndSA;              // End temperature for simulated annealing
    double d_bigM;                   // Big m for the MIP
    double d_tolerance;              // Tolerance for floating point calculations

    unsigned short d_randomSeed;               // Random seed
    unsigned short d_nConstructionTries;       // Number of tries for construction heuristic    
    unsigned short d_nDestroyOps;              // Number of destroy operations
    unsigned short d_nRepairOps;               // Number of repair operations
    unsigned short d_maxNeighborhoodSize;      // Maximum size of the neighborhood
    unsigned short d_sortType;                 // Type of node sorting before construction heuristic
    unsigned short d_nColsMaxExact;            // Maximum number of columns per pricing iteration
    unsigned short d_nColsMaxALNS;             // Maximum number of columns per pricing iteration
    unsigned short d_nSolsInit;                // Initial number of columns per pricing iteration
    unsigned short d_nRestartALNS;             // Number of restarts for the ALNS
    unsigned int d_nALNSIterationsPerNode;     // Number of iterations for the Metaheuristic
    unsigned int d_nALNSInitIterationsPerNode; // Number of initialization ALNS iterations for the Metaheuristic
    unsigned int d_nCheckFeasible;             // Number of iterations before checking feasibility
    unsigned int d_timeLimit;                  // Time limit for the pricing algorithms, in seconds
    
    bool d_verbose;             // Whether to print verbose output
    bool d_writeFiles;          // Whether to write the solution and logs to file
    bool d_simulatedAnnealing;  // Whether to use simulated annealing
    bool d_alnsInit;            // Whether to initialize the MIP with the ALNS solution
    bool d_verifyExact;         // Whether to verify the ALNS solution with the exact pricing algorithm
    bool d_writeSols;           // Whether to write the solutions to file
    
    // Default parameters
    Settings();
    bool load(const std::string path_settings);
};

#endif // SETTINGS_H