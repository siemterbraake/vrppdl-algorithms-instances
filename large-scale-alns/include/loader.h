/*
Loader functions for the metaheuristic, loads the instance files and creates the data structures.

Written by: Casper Bazelmans and Siem ter Braake, 2024
*/

#ifndef LOADER_H
#define LOADER_H


#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <limits>
#include <cmath>

struct VehicleType {
    float d_costFixed;
    float d_costVariable;
    float d_speed;              // km/s, for easy conversion to seconds
    int d_timeMax;              // s
    int d_timeDriveMax;         // s
    std::size_t d_idVehicleType;
    std::size_t d_capacity;     // kg
    std::size_t d_alpha;        // kg
    std::size_t d_beta;         // kg


    friend std::istream &operator >>(std::istream &in, VehicleType &obj);
    friend std::ostream &operator <<(std::ostream &out, VehicleType &obj);
};

struct VehicleTypeInfo {
    std::size_t d_idVehicleType;
    std::size_t d_idDepot;
    std::size_t d_nVehicles;

    friend std::istream &operator >>(std::istream &in, VehicleTypeInfo &obj);
    friend std::ostream &operator <<(std::ostream &out, VehicleTypeInfo &obj);
};

struct ScheduledLine {
    float d_costRequest;
    int d_timeDep;
    int d_timeArr;
    std::size_t d_idFrom;
    std::size_t d_idTo;
    std::size_t d_capacity;

    friend std::istream &operator >>(std::istream &in, ScheduledLine &obj);
    friend std::ostream &operator <<(std::ostream &out, ScheduledLine &obj);
};

struct Node {
    double d_latitude;    
    double d_longitude;
    int d_timeServ;             // Service time
    int d_twDepotStart;         // Time window start at depot    
    int d_twDepotEnd;           // Time window end at depot
    int d_twCustStart;          // Time window start at customer
    int d_twCustEnd;            // Time window end at customer
    std::size_t d_idNode;       // Node ID
    std::size_t d_idDepot;      // Incoming / Outgoing depot ID
    std::size_t d_idDepotAlloc; // Allocated depot for delivery / collection
    std::size_t d_pc;           // Postal code
    short d_type;               // -1 = depot, 0 = collection, 1 = delivery
    std::size_t d_qDel;         // Delivery quantity
    std::size_t d_qCol;         // Collection quantity


    friend std::istream &operator >>(std::istream &in, Node &obj);
    friend std::ostream &operator <<(std::ostream &out, Node &obj);
};

struct Instance {
    std::vector<std::vector<float>> d_nodeEmbeddings; // Node embeddings
    std::map<std::tuple<std::size_t, std::size_t>, std::vector<std::size_t>> d_slMap; // Map of scheduled lines by depot
    
    std::vector<Node> d_nodes;                        // Nodes
    std::vector<VehicleType> d_vehTypes;              // Vehicle types
    std::vector<VehicleTypeInfo> d_vehTypesInfo;      // Vehicle type info
    std::vector<ScheduledLine> d_scheduledLines;      // Scheduled lines
    
    std::string d_name;          // Name of the instance
    int d_timeHorizonMax;        // Maximum time horizon    
    std::size_t d_nVehTypes;     // Number of vehicle types 
    std::size_t d_nVehTypesInfo; // Number of vehicle type info
    std::size_t d_nSL;           // Number of scheduled lines   
    std::size_t d_nNodes;        // Number of nodes
    std::size_t d_nDepots;       // Number of depots
    std::size_t d_nCustNodes;    // Number of customer nodes

    Instance();
    bool load(std::string instPath);
    void addAllocatedDepots(std::string tallPath);
    
    friend std::ostream &operator <<(std::ostream &out, Instance &obj);
};

#endif  // LOADER_H