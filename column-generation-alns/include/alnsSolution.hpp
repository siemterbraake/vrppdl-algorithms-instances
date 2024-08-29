/*
Solution structs for the metaheuristic, creates a solution object based on the instance.

Written by: Siem ter Braake, 2024
*/

#ifndef SOLUTION_H
#define SOLUTION_H

#include "instance.hpp"
#include "graph.hpp"
#include "route.hpp"
#include "visit.hpp"
#include "duals.hpp"
#include "label.hpp"

#include <random>
#include <vector>
#include <limits>
#include <iostream>
#include <cmath>
#include <unordered_map>
#include <set>
#include <deque>

struct VehicleTypeSolution {
    /*
    Struct that stores the solution for a single vehicle type.
    */
    // Parameters
    std::vector<Route> d_routes; // Routes of the solution for each vehicle type
    unsigned short d_idVehicleType; // Id of the vehicle type
    unsigned short d_idDepot;       // Id of the depot
    unsigned short d_nVehicles;     // Number of vehicles available
    unsigned short d_nUsedVehicles; // Number of vehicles used

    // Functions
    VehicleTypeSolution();
    VehicleTypeSolution(unsigned short idVehicleType, unsigned short idDepot, unsigned short nVehicles);
    double checkOpen(const Node &req, int twDepotStart, int twDepotEnd, const Graph &graph, const VehicleType &vehType) const;
    void openRoute(const Node &req, int twDepotStart, int twDepotEnd,
                   const Node &depot, const Graph &graph, const VehicleType &vehType);
    void closeRoute(unsigned short idRoute, std::unordered_map<unsigned short, VisitLoc> &visitLocs);
};

struct DepotSolution {
    /*
    Struct that stores the solution for a single depot.
    */
    // Parameters
    unsigned short d_idDepot;
    unsigned short d_nVehicleTypes;
    std::vector<VehicleTypeSolution> d_vehTypeSols;

    // Functions
    DepotSolution();
    DepotSolution(unsigned short idDepot, std::vector<VehicleTypeInfo> vehTypesInfo);
};

struct SLSolution {
    /*
    Struct that stores the solution for a single scheduled line.
    */
    // Parameters
    unsigned short d_idSL;
    unsigned short d_qAssigned;
    unsigned short d_nAssigned;
    std::vector<unsigned short> d_assignments;

    // Functions
    SLSolution();
    SLSolution(unsigned short idSL);
    void assign(const Node &req);
    void remove(const Node &req, std::unordered_map<unsigned short, VisitLoc> &visitLocs);
}; 

struct ALNSSolution {
    // Parameters
    double d_cost;
    std::vector<DepotSolution> d_depotSols;                // Routes of the solution for each depot (depot, route)
    std::vector<SLSolution> d_SLSols;                      // Maps the order to the scheduled line index (order, SL)
    std::vector<unsigned short> d_reqUnassigned;              // vector of unassigned requests
    std::unordered_map<unsigned short, VisitLoc> d_visitLocs; // Maps the node id to the location in the solution (node, location)
    const Instance *d_pInst;                               // Pointer to the instance
    const Graph *d_pGraph;                                 // Pointer to the graph
    const Settings *d_pSettings;                           // Pointer to the settings

    // Core Functions -> Solution.cpp
    ALNSSolution();
    ALNSSolution(const Instance &inst, const Graph &graph, const Settings &settings);
    void sortRequests(unsigned short logic);
    bool insertNode(const Node &req, unsigned short pos, unsigned short &idDepot, unsigned short &idVehicleType, unsigned short &idRoute, unsigned short &idSL, int twDepotStart, int twDepotEnd, double costInsert);
    void checkOpenRouteGreedy(const Node &req, const Duals &duals, bool noise, unsigned short &bestDepot, unsigned short &bestVehicle, int &bestTwDepotStart, int &bestTwDepotEnd, unsigned short &bestSL, double &costOpen, std::mt19937 &gen);
    void removeNode(unsigned short idNode, const Duals &duals);
    void removeRoute(unsigned short idDepot, unsigned short idVehicleType, unsigned short idRoute, unsigned short &ctrRemoved);
    void selectSL(unsigned short &SL, bool &found, int &twDepotStart, int &twDepotEnd, const Node &req, unsigned short idDepot) const;
    void writeSLUsage(std::ofstream &file) const;
    void writeCsv(std::string path) const;
    std::deque<Label> getLabels() const;
    std::size_t hash() const;

    // Checker functions -> SolutionChecker.cpp
    bool isValid(bool destroyed, Duals duals) const;
    bool isFeasible() const;

    // Comparison operator for sorting
    bool operator<(const ALNSSolution& other) const {
        return d_cost < other.d_cost;
    }
};

#endif // SOLUTION_H