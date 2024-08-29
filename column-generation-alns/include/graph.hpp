/*
Graph functions for the metaheuristic, defines the graph structs and defines function for determination of graph
as well as time calculation.

Written by: Casper Bazelmans and Siem ter Braake, 2024
*/

#ifndef GRAPH_H
#define GRAPH_H

#include "instance.hpp"
#include "settings.hpp"

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
    std::vector<double> d_distMatrix;                      // Distance matrix for each vehicle type (nVehicle, nNode, nNode in a single column)
    std::vector<int>  d_timeMatrix;                        // Time matrix for each vehicle type (nVehicle, nNode, nNode in a single column)
    std::vector<unsigned short> d_nearestNodes;               // List of nearest nodes for each node (nNode, nNearNeighbors in a single column)
    double d_maxDist;                                      // Largest distance value
    unsigned short d_nNodes;                                  // Number of nodes
    unsigned short d_nNearestNodes;                           // Number of near neigbors for each node
    unsigned short d_nVehicleTypes;                           // Number of vehicle types

    Graph() {};
    void load(const Instance &inst, Settings &settings);

    void calcDima(const Instance &inst, const Settings &settings);
    void calcNearestNodes(const Instance &inst, Settings &settings);
    double calcHaversineDist(double lat1, double lon1, double lat2, double lon2) const;
    double calcEuclideanDist(double x1, double y1, double x2, double y2) const;
    double calcSpacioTemporalDist(const Node &n1, const Node &n2, double w1, double w2) const;
    void writeCSV(const Settings &settings);
    void removeDetours();
    void setDist(unsigned short idVehicleType, unsigned short idNode1, unsigned short idNode2, double dist);
    void setTime(unsigned short idVehicleType, unsigned short idNode1, unsigned short idNode2, int time);
    void setNeighbors(unsigned short idNode, std::vector<unsigned short> nearestNodes);

    inline double getDist(unsigned short idVehicleType, unsigned short idNode1, unsigned short idNode2) const {
        return d_distMatrix[d_nNodes * d_nNodes * idVehicleType + d_nNodes * idNode1 + idNode2];
    }

    inline int getTime(unsigned short idVehicleType, unsigned short idNode1, unsigned short idNode2) const {
        return d_timeMatrix[d_nNodes * d_nNodes * idVehicleType + d_nNodes * idNode1 + idNode2];
    }

    inline std::pair<std::vector<unsigned short>::const_iterator, std::vector<unsigned short>::const_iterator> getNearestNodes(unsigned short idNode) const {
        return {d_nearestNodes.cbegin() + static_cast<int>(idNode * d_nNearestNodes), d_nearestNodes.cbegin() + static_cast<int>((idNode + 1) * d_nNearestNodes)};
    }
};

#endif // GRAPH_H