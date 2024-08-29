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
    unsigned short d_idDepot;        // Depotsolution id
    unsigned short d_idSL;           // Scheduled line index
    unsigned short d_posSL;          // Position in the scheduled line
    unsigned short d_idVehicleType;  // Vehicletypesolution id
    unsigned short d_idRoute;        // Route id
    unsigned short d_idVisit;        // Visit id

    // Functions
    VisitLoc();
    VisitLoc(unsigned short idDepot, unsigned short idSL, unsigned short posSL, unsigned short idVehicleType, unsigned short idRoute, unsigned short idVisit);
    std::tuple<unsigned short,unsigned short,unsigned short> getRoute();
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
    unsigned short d_idVisit; // Visit id, ordered 0 and last index are depot visits
    unsigned short d_idNode;  // Node being visited
    unsigned short d_qDel;    // This is measured at the time of departure, so excludes the quantity delivered at the node
    unsigned short d_qCol;    // This is measured at the time of departure, so includes the quantity collected at the node
    short d_type;          // Type of visit, 0 = depot, 1 = delivery, 2 = collection

    // Functions
    Visit();
    Visit(int timeArrival, int timeDeparture, int twWait, int twSlackStart, int twSlackEnd, 
          unsigned short idVisit, unsigned short idNode, unsigned short qDel, unsigned short qCol, short type);
};

#endif