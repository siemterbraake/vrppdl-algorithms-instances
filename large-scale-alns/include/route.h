/*
Route functions and data structures, used to represent and optimize a solution.

Written by: Siem ter Braake, 2024
*/

#ifndef ROUTE_H
#define ROUTE_H

#include "loader.h"
#include "graph.h"
#include "visit.h"

#include <vector>

struct Route {
    /*
    Struct that stores a route from and to a depot using a vehicle.
    */
    // Parameters
    std::vector<Visit> d_visits; // Ordered list of visits
    float d_cost;                // Current route costs
    float d_dist;                // Current route distance
    int d_timeDeparture;         // Actual departure time of the route
    int d_timeArrival;           // Actual arrival time of the route
    int d_timeDrive;             // Total time spent driving
    int d_timeTravel;            // Time away from the depot
    std::size_t d_idRoute;    
    std::size_t d_idVehicleType;
    std::size_t d_idDepot;
    std::size_t d_nVisits;       // Number of visits
    std::size_t d_loadMax;       // Maximum load in the vehicle along the route
    std::size_t d_iMix;          // First visit index where collections are feasible
    std::size_t d_iFill;         // Last visit index where deliveries are feasible

    // Functions
    Route();
    Route(std::size_t idRoute, std::size_t idVehicleType, std::size_t idDepot);
    std::pair<std::size_t, float> checkGreedyInsert(const Node &req, int twDepotStart, 
                                             int twDepotEnd, const Graph &graph, 
                                             const VehicleType &vehType) const;
    void insert(const Node &req, std::size_t pos, int twDepotStart,
                int twDepotEnd, const Graph &graph, const VehicleType &vehType, 
                std::unordered_map<std::size_t, VisitLoc> &visitLocs);
    void remove(const Node &req, std::size_t pos, const Graph &graph, const VehicleType &vehType, 
                const Instance &inst, std::unordered_map<std::size_t, VisitLoc> &visitLocs);
    void removeUnwantedSlack();
    void updateLoadHelpers(const VehicleType &vehType);
};

#endif