/*
Graph functions for the metaheuristic, defines the graph structs and defines function for determination of graph
as well as time calculation.

Written by: Casper Bazelmans and Siem ter Braake, 2024
*/

#ifndef GRAPH_H
#define GRAPH_H

#include "loader.h"
#include "settings.h"

#include <cmath>
#include <chrono>
#include <algorithm>
#include <vector>
#include <numeric>
#include <filesystem>

#include <iostream>
#include <fstream>
#include <sstream>


struct Graph {
    std::vector<float> d_distMatrix;                       // Distance matrix for each vehicle type (nVehicle, nNode, nNode in a single column)
    std::vector<int>  d_timeMatrix;                        // Time matrix for each vehicle type (nVehicle, nNode, nNode in a single column)
    std::vector<std::size_t> d_nearestNodes;               // List of nearest nodes for each node (nNode, nNearNeighbors in a single column)
    float d_maxDist;                                       // Largest distance value
    std::size_t d_nNodes;                                  // Number of nodes
    std::size_t d_nNearestNodes;                           // Number of near neigbors for each node
    std::size_t d_nVehicleTypes;                           // Number of vehicle types

    Graph() {};
    void load(const Instance &inst, Settings &settings);

    void calcDima(const Instance &inst, const Settings &settings);
    void calcNearestNodes(const Instance &inst, Settings &settings);
    void loadMatrix();
    void writeCSV(const Settings &settings);
    void removeDetours();
    void setDist(std::size_t idVehicleType, std::size_t idNode1, std::size_t idNode2, float dist);
    void setTime(std::size_t idVehicleType, std::size_t idNode1, std::size_t idNode2, int time);
    void setNeighbors(std::size_t idNode, std::vector<std::size_t> nearestNodes);
    float getDist(std::size_t idVehicleType, std::size_t idNode1, std::size_t idNode2) const;
    int getTime(std::size_t idVehicleType, std::size_t idNode1, std::size_t idNode2) const;
    std::pair<std::vector<std::size_t>::const_iterator, std::vector<std::size_t>::const_iterator> getNearestNodes(std::size_t idNode) const; 

    float calcHaversineDist(double lat1, double lon1, double lat2, double lon2) const;
    float calcEuclideanDist(double x1, double y1, double x2, double y2) const;
    float calcSpacioTemporalDist(const Node &n1, const Node &n2, float w1, float w2) const;
};

#endif // GRAPH_H