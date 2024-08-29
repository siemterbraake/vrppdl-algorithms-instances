/*
Solution structs for the metaheuristic, creates a solution object based on the instance.

Written by: Siem ter Braake, 2024
*/

#ifndef SOLUTION_H
#define SOLUTION_H

#include "loader.h"
#include "graph.h"
#include "route.h"
#include "visit.h"

#include <random>
#include <vector>
#include <limits>
#include <iostream>
#include <cmath>
#include <unordered_map>
#include <set>

struct VehicleTypeSolution {
    /*
    Struct that stores the solution for a single vehicle type.
    */
    // Parameters
    std::vector<Route> d_routes; // Routes of the solution for each vehicle type
    float d_penalty;             // Penalty for excessive vehicles used
    std::size_t d_idVehicleType; // Id of the vehicle type
    std::size_t d_idDepot;       // Id of the depot
    std::size_t d_nVehicles;     // Number of vehicles available
    std::size_t d_nUsedVehicles; // Number of vehicles used

    // Functions
    VehicleTypeSolution();
    VehicleTypeSolution(std::size_t idVehicleType, std::size_t idDepot, std::size_t nVehicles);
    float checkOpen(const Node &req, int twDepotStart, int twDepotEnd, float excessiveRoutePenalty,
                    const Graph &graph, const VehicleType &vehType) const;
    void openRoute(const Node &req, int twDepotStart, int twDepotEnd, float excessiveRoutePenalty,
                   const Node &depot, const Graph &graph, const VehicleType &vehType);
    void closeRoute(std::size_t idRoute, float excessiveRoutePenalty, std::unordered_map<std::size_t, VisitLoc> &visitLocs);
};

struct DepotSolution {
    /*
    Struct that stores the solution for a single depot.
    */
    // Parameters
    std::size_t d_idDepot;
    std::size_t d_nVehicleTypes;
    std::vector<VehicleTypeSolution> d_vehTypeSols;

    // Functions
    DepotSolution();
    DepotSolution(std::size_t idDepot, std::vector<VehicleTypeInfo> vehTypesInfo);
};

struct SLSolution {
    /*
    Struct that stores the solution for a single scheduled line.
    */
    // Parameters
    std::size_t d_idSL;
    std::size_t d_qAssigned;
    std::size_t d_nAssigned;
    std::vector<std::size_t> d_assignments;

    // Functions
    SLSolution();
    SLSolution(std::size_t idSL);
    void assign(const Node &req);
    void remove(const Node &req, std::unordered_map<std::size_t, VisitLoc> &visitLocs);
}; 

struct Solution {
    // Parameters
    float d_cost;
    std::vector<DepotSolution> d_depotSols;                // Routes of the solution for each depot (depot, route)
    std::vector<SLSolution> d_SLSols;                      // Maps the order to the scheduled line index (order, SL)
    std::vector<std::size_t> d_reqUnassigned;              // vector of unassigned requests
    std::vector<std::size_t> d_reqInfeasible;              // vector of infeasible requests
    std::unordered_map<std::size_t, VisitLoc> d_visitLocs; // Maps the node id to the location in the solution (node, location)
    std::vector<std::vector<std::size_t>> d_subproblems;   // Subproblems for the ML model
    const Instance *d_pInst;                               // Pointer to the instance
    const Graph *d_pGraph;                                 // Pointer to the graph
    const Settings *d_pSettings;                           // Pointer to the settings

    // Core Functions -> Solution.cpp
    Solution();
    Solution(const Instance &inst, const Graph &graph, const Settings &settings);
    void sortRequests(std::size_t logic);
    void insertNode(const Node &req, std::size_t pos, std::size_t idDepot, std::size_t idVehicleType, std::size_t idRoute, std::size_t idSL, int twDepotStart, int twDepotEnd, float costInsert);
    void checkOpenRouteGreedy(const Node &req, bool noise, bool restrictServiceArea, std::size_t &bestDepot, std::size_t &bestVehicle, int &bestTwDepotStart, int &bestTwDepotEnd, std::size_t &bestSL, float &costOpen, std::mt19937 &gen);
    void removeNode(std::size_t idNode);
    void removeRoute(std::size_t idDepot, std::size_t idVehicleType, std::size_t idRoute, std::size_t &ctrRemoved);
    void selectSL(std::size_t &SL, bool &found, int &twDepotStart, int &twDepotEnd, const Node &req, std::size_t idDepot) const;
    void patchCosts();
    std::vector<std::size_t> constructSubproblem(const Route &route, std::size_t size) const;
    void updateSubproblems();
    float getPenalty();
    std::size_t getNAreaInfeasible() const;
    void writeSLUsage(std::ofstream &file) const;
    void writeCsv(std::string path) const;
    void writeJSON(std::string path) const;

    // Checker functions -> SolutionChecker.cpp
    bool isValid(bool destroyed) const;
    bool isFeasible() const;
};

#endif // SOLUTION_H