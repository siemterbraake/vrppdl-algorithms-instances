/*
The label struct stores the information of a partial path in the graph.

Written by: Casper Bazelmans, 2024
*/

#ifndef LABEL_H
#define LABEL_H

#include <cassert>
#include <vector>
#include <iostream>
#include <algorithm>
#include <functional>

struct Label
{
    /*
    Label represents a partial path in the graph
    */
    bool d_hasTw;                // Current partial route has time windows
    unsigned short d_idNode;           // Current node (max 65,535 nodes)
    unsigned short d_idVehType;        // Current vehicle (max 65,535 vehicles)
    unsigned short d_idDepot;          // Current depot (max 65,535 depots)
    std::vector<unsigned short> d_visitedNodes; // Nodes visited
    unsigned short d_qtyCanPickup;                      // Can pickup qty
    unsigned short d_qtyCanDeliver;                      // Can deliver qty
    int d_depTimeEarliest;             // Earliest departure time at node
    int d_depTimeLatest;             // Latest departure time at node
    int d_depTimeLatestDepot;             // Latest departure time at depot
    int d_arrTimeEarliestDepot;             // Earliest arrival time at depot
    int d_curRouteTime;             // Route duration until departure at current node
    int d_curDriveTime;             // Driving duration until current node
    double d_cost;                // Reduced cost of partial route
    int d_startTime;    // (optimal) start time of the route within its dynamic tw's
    std::vector<unsigned char> d_sls; // Keeps sl idx of every node in route for current label 

    Label();
    Label(
        bool hastw,              
        unsigned short idNode,
        unsigned short idVehicle,
        unsigned short idDepot,                
        std::vector<unsigned short> visitedNodes,
        unsigned short qtyCanPickup,                
        unsigned short qtyCanDeliver,                
        int depTimeEarliest,       
        int depTimeLatest,     
        int depTimeLatestDepot,      
        int arrTimeEarliestDepot,      
        int curRouteTime,
        int curDriveTime,                     
        double cost,
        int startTime,
        std::vector<unsigned char> sls
    );
    std::size_t hash() const;

    // Define the equality operator
    bool operator==(const Label& other) const {
        return d_idVehType == other.d_idVehType &&
               d_visitedNodes == other.d_visitedNodes &&
               d_sls == other.d_sls;
    }
    // Comparison operator for sorting
    bool operator<(const Label& other) const {
        return d_cost < other.d_cost;
    }
};

inline bool compareReducedCost(Label const &label1, Label const &label2)
{
    return label1.d_cost < label2.d_cost;
}

inline bool isSubsetOrEqual(std::vector<unsigned short> const& v1, std::vector<unsigned short> const& v2)
{
    /*
    Check if v1 is subset of or equal to v2. 0.41 seconds for 1 million times.
    */
    for (auto const& av : v1)
    {
        if (std::find(v2.begin(), v2.end(), av) == v2.end())
        {
            return false;
        }
    }

    return true;
}

inline bool dominates(const Label& label1, const Label& label2)
{
    assert(label1.d_idNode == label2.d_idNode);

    return (label1.d_cost <= label2.d_cost) &&
           (label1.d_qtyCanPickup >= label2.d_qtyCanPickup) &&
           (label1.d_qtyCanDeliver >= label2.d_qtyCanDeliver) &&
           (label1.d_arrTimeEarliestDepot <= label2.d_arrTimeEarliestDepot) &&
           (label1.d_depTimeLatestDepot >= label2.d_depTimeLatestDepot) &&
           (label1.d_curRouteTime <= label2.d_curRouteTime) &&
           isSubsetOrEqual(label1.d_visitedNodes, label2.d_visitedNodes);
}

inline void printLabel(const Label &label)
{
    std::cout << "Label Details:\n";
    std::cout << "  has_tw: " << (label.d_hasTw ? "true" : "false") << "\n";
    std::cout << "  n: " << label.d_idNode << "\n";
    std::cout << "  N: [";

    for (unsigned short i = 0; i < label.d_visitedNodes.size(); ++i)
    {
        std::cout << label.d_visitedNodes[i];

        if (i < label.d_visitedNodes.size() - 1)
        {
            std::cout << ", ";
        }
    }
    std::cout << "]\n";
    std::cout << "  p: " << label.d_qtyCanPickup << "\n";
    std::cout << "  d: " << label.d_qtyCanDeliver << "\n";
    std::cout << "  e: " << label.d_depTimeEarliest << "\n";
    std::cout << "  l: " << label.d_depTimeLatest << "\n";
    std::cout << "  b: " << label.d_depTimeLatestDepot << "\n";
    std::cout << "  a: " << label.d_arrTimeEarliestDepot << "\n";
    std::cout << "  o: " << label.d_curRouteTime << "\n";
    std::cout << "  m: " << label.d_curDriveTime << "\n";
    std::cout << "  cost: " << label.d_cost << "\n";
    std::cout << "  start_time: " << label.d_startTime << "\n";
    std::cout << "  sls: [";

    for (unsigned short i = 0; i < label.d_sls.size(); ++i)
    {
        std::cout << static_cast<int>(label.d_sls[i]);

        if (i < label.d_sls.size() - 1)
        {
            std::cout << ", ";
        }
    }
    std::cout << "]\n";
}

#endif // LABEL_H