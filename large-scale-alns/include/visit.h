/*
Visit data structures, used to represent a solution. 

Written by: Siem ter Braake, 2024
*/

#ifndef VISIT_H
#define VISIT_H

#include <tuple>
#include <limits>

struct VisitLoc {
    /*
    Struct that stores the location of a node.
    */
    // Parameters
    std::size_t d_idDepot;        // Depotsolution id
    std::size_t d_idSL;           // Scheduled line index
    std::size_t d_posSL;          // Position in the scheduled line
    std::size_t d_idVehicleType;  // Vehicletypesolution id
    std::size_t d_idRoute;        // Route id
    std::size_t d_idVisit;        // Visit id

    // Functions
    VisitLoc();
    VisitLoc(std::size_t idDepot, std::size_t idSL, std::size_t posSL, std::size_t idVehicleType, std::size_t idRoute, std::size_t idVisit);
    std::tuple<std::size_t,std::size_t,std::size_t> getRoute();
};

struct Visit {
    /*
    Struct that stores an individual visit to a customer node.
    */
    // Parameters
    int d_timeArrival;     // Time of arrival at the node
    int d_timeDeparture;   // Time of departure at the node
    int d_twWait;          // Time waited before visiting the node, this time comes before the timeArrival
    int d_twSlackStart;    // Time between the timedeparture and the end of the time window
    int d_twSlackEnd;      // Time between the start of the time window and the timearrival
    std::size_t d_idVisit; // Visit id, ordered 0 and last index are depot visits
    std::size_t d_idNode;  // Node being visited
    std::size_t d_qDel;    // This is measured at the time of departure, so excludes the quantity delivered at the node
    std::size_t d_qCol;    // This is measured at the time of departure, so includes the quantity collected at the node
    short d_type;          // Type of visit, 0 = depot, 1 = delivery, 2 = collection

    // Functions
    Visit();
    Visit(int timeArrival, int timeDeparture, int twWait, int twSlackStart, int twSlackEnd, 
          std::size_t idVisit, std::size_t idNode, std::size_t qDel, std::size_t qCol, short type);
};

#endif