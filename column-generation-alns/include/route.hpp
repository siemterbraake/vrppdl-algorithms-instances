/*
Route functions and data structures, used to represent and optimize a solution.

Written by: Siem ter Braake, 2024
*/

#ifndef ROUTE_H
#define ROUTE_H

#include "instance.hpp"
#include "graph.hpp"
#include "visit.hpp"
#include "label.hpp"
#include "duals.hpp"

#include <vector>

struct Route {
    /*
    Struct that stores a route from and to a depot using a vehicle.
    */
    // Parameters
    std::vector<Visit> d_visits; // Ordered list of visits
    double d_cost;                // Current route costs
    double d_dist;                // Current route distance
    int d_timeDeparture;         // Actual departure time of the route
    int d_timeArrival;           // Actual arrival time of the route
    int d_timeDrive;             // Total time spent driving
    int d_timeTravel;            // Time away from the depot
    unsigned short d_idRoute;    
    unsigned short d_idVehicleType;
    unsigned short d_idDepot;
    unsigned short d_nVisits;       // Number of visits
    unsigned short d_loadMax;       // Maximum load in the vehicle along the route
    unsigned short d_iMix;          // First visit index where collections are feasible
    unsigned short d_iFill;         // Last visit index where deliveries are feasible

    // Functions
    Route();
    Route(unsigned short idRoute, unsigned short idVehicleType, unsigned short idDepot);
    std::pair<unsigned short, double> checkGreedyInsert(const Node &req, int twDepotStart, 
                                             int twDepotEnd, const Graph &graph, 
                                             const VehicleType &vehType) const;
    void insert(const Node &req, unsigned short pos, int twDepotStart,
                int twDepotEnd, const Graph &graph, const VehicleType &vehType, 
                std::unordered_map<unsigned short, VisitLoc> &visitLocs);
    void remove(const Node &req, unsigned short pos, const Graph &graph, const VehicleType &vehType, 
                const Instance &inst, std::unordered_map<unsigned short, VisitLoc> &visitLocs);
    void removeUnwantedSlack();
    void updateLoadHelpers(const VehicleType &vehType);
    Label toLabel(const Instance& inst, const std::unordered_map<unsigned short, VisitLoc>& visitLocs) const;
};

#endif