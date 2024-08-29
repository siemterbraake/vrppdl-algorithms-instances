/*
The loader struct is responsible for loading the instance data from a file.

Written by: Casper Bazelmans and Siem ter Braake, 2024
*/

#ifndef H_LOADER
#define H_LOADER

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <map>
#include <tuple>

// General function(s)
std::string get_current_instance(std::string path_to_inst);
double get_haversine_distance(double lat1, double lon1, double lat2, double lon2);

// Data structure to store data loaded from instance
struct VehicleType
{
    unsigned short d_idVehicleType = 0;
    double d_costFixed = 0.0;
    double d_costVariable = 0.0;
    double d_speed = 0.0;    // km/s for easier calculations
    unsigned short d_capacity = 0;      // kg
    unsigned short d_alpha = 0;       // kg
    unsigned short d_beta = 0;        // kg
    int d_timeMax = 0;       // s
    int d_timeDriveMax = 0;

    friend std::istream &operator >>(std::istream &in, VehicleType &obj);
    friend std::ostream &operator <<(std::ostream &out, VehicleType &obj);
};


struct VehicleTypeInfo
{
    unsigned short d_idVehicleType = 0;
    unsigned short d_idDepot = 0;
    unsigned short d_nVehicles = 0;

    friend std::istream &operator >>(std::istream &in, VehicleTypeInfo &obj);
    friend std::ostream &operator <<(std::ostream &out, VehicleTypeInfo &obj);
};


struct ScheduledLine
{
    unsigned short d_idFrom = 0;
    unsigned short d_idTo = 0;
    double d_costRequest = 0;
    unsigned short d_capacity = 0;
    int d_timeDep = 0;
    int d_timeArr = 0;

    friend std::istream &operator >>(std::istream &in, ScheduledLine &obj);
    friend std::ostream &operator <<(std::ostream &out, ScheduledLine &obj);
};


struct Node
{
    unsigned short d_idNode = 0;
    unsigned short d_idDepot = 0;
    double d_latitude = 0.0;
    double d_longitude = 0.0;
    short d_type = 0;
    int d_timeServ = 0;      // To match other time variable types
    unsigned short d_qDel = 0;
    unsigned short d_qCol = 0;
    int d_twDepotStart = 0;
    int d_twDepotEnd = 0;
    int d_twCustStart = 0;
    int d_twCustEnd = 0;

    friend std::istream &operator >>(std::istream &in, Node &obj);
    friend std::ostream &operator <<(std::ostream &out, Node &obj);
};


struct Instance
{
    std::string d_name;;
    unsigned short d_nVehTypes;
    unsigned short d_nVehTypesInfo;
    unsigned short d_nSL;
    unsigned short d_nNodes;
    unsigned short d_nDepots;
    unsigned short d_nCustNodes;
    int d_maxTimeHorizon;
    
    std::vector<VehicleType> d_vehTypes;
    std::vector<VehicleTypeInfo> d_vehTypesInfo;
    std::vector<ScheduledLine> d_scheduledLines;
    std::vector<Node> d_nodes;

    std::map<std::tuple<unsigned short, unsigned short>, std::vector<unsigned short>> d_slMap; // Map of scheduled lines by depot

    Instance() : d_name("None"), d_nVehTypes(0), d_nVehTypesInfo(0), d_nSL(0), 
                    d_nNodes(0), d_nDepots(0), d_nCustNodes(0), d_maxTimeHorizon(0) {}

    bool load(const std::string &path_to_instance);
    
    friend std::ostream &operator <<(std::ostream &out, Instance &obj);
};


#endif  // H_LOADER