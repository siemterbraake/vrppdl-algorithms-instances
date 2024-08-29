#include "label.hpp"

Label::Label()
{
    d_hasTw = false;
    d_idNode = 0;
    d_idVehType = 0;
    d_idDepot = 0;
    d_visitedNodes = {};
    d_qtyCanPickup = 0;
    d_qtyCanDeliver = 0;
    d_depTimeEarliest = 0;
    d_depTimeLatest = 0;
    d_depTimeLatestDepot = 0;
    d_arrTimeEarliestDepot = 0;
    d_curRouteTime = 0;
    d_curDriveTime = 0;
    d_cost = 0;
    d_startTime = 0;
    d_sls = {};
}

Label::Label(
    bool hastw,              
    unsigned short idNode,
    unsigned short idVehType,
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
) {
    d_hasTw = hastw;
    d_idNode = idNode;
    d_idVehType = idVehType;
    d_idDepot = idDepot;
    d_visitedNodes = visitedNodes;
    d_qtyCanPickup = qtyCanPickup;
    d_qtyCanDeliver = qtyCanDeliver;
    d_depTimeEarliest = depTimeEarliest;
    d_depTimeLatest = depTimeLatest;
    d_depTimeLatestDepot = depTimeLatestDepot;
    d_arrTimeEarliestDepot = arrTimeEarliestDepot;
    d_curRouteTime = curRouteTime;
    d_curDriveTime = curDriveTime;
    d_cost = cost;
    d_startTime = startTime;
    d_sls = sls;
}

std::size_t Label::hash() const
{
    std::size_t hashValue = 0;
    std::hash<unsigned short> hasher;
    // Hash the depot id and the vehicle type id
    hashValue ^= hasher(d_idDepot) + 0x9e3779b9 + (hashValue << 6) + (hashValue >> 2);
    hashValue ^= hasher(d_idVehType) + 0x9e3779b9 + (hashValue << 6) + (hashValue >> 2);
    // Hash the visited nodes
    for (unsigned short elem : d_visitedNodes) {
        hashValue ^= hasher(elem) + 0x9e3779b9 + (hashValue << 6) + (hashValue >> 2);
    }
    // Hash the sls
    for (unsigned char elem : d_sls) {
        unsigned short elemShort = static_cast<unsigned short>(elem);
        hashValue ^= hasher(elemShort) + 0x9e3779b9 + (hashValue << 6) + (hashValue >> 2);
    }
    return hashValue;
}
