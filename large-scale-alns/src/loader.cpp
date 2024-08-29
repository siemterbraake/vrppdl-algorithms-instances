#include "loader.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>


std::istream &operator>>(std::istream &in, VehicleType &obj) {
    in >> 
    obj.d_idVehicleType >>
    obj.d_costFixed >>
    obj.d_costVariable >>
    obj.d_speed >>
    obj.d_capacity >>
    obj.d_alpha >>
    obj.d_beta >>
    obj.d_timeMax >>
    obj.d_timeDriveMax;
    return in;
}

std::ostream &operator <<(std::ostream &out, VehicleType &obj) {
    out <<
    obj.d_idVehicleType << ", " <<
    obj.d_costFixed << ", " <<
    obj.d_costVariable << ", " <<
    obj.d_speed << ", " <<
    obj.d_capacity << ", " <<
    obj.d_alpha << ", " <<
    obj.d_beta << ", " <<
    obj.d_timeMax << ", " <<
    obj.d_timeDriveMax;
    return out;
}

std::istream &operator>>(std::istream &in, VehicleTypeInfo &obj) {
    in >> 
    obj.d_idVehicleType >>
    obj.d_idDepot >>
    obj.d_nVehicles;
    return in;
}

std::ostream &operator <<(std::ostream &out, VehicleTypeInfo &obj) {
    out <<
    obj.d_idVehicleType << ", " <<
    obj.d_idDepot << ", " <<
    obj.d_nVehicles;
    return out;
}


std::istream &operator>>(std::istream &in, ScheduledLine &obj) {
    in >>
    obj.d_idFrom >>
    obj.d_idTo >>
    obj.d_costRequest >>
    obj.d_capacity >>
    obj.d_timeDep >>
    obj.d_timeArr;
    return in;
}

std::ostream &operator <<(std::ostream &out, ScheduledLine &obj) {
    out <<
    obj.d_idFrom << ", " <<
    obj.d_idTo << ", " <<
    obj.d_costRequest << ", " <<
    obj.d_capacity << ", " <<
    obj.d_timeDep << ", " <<
    obj.d_timeArr;
    return out;
}


std::istream &operator>>(std::istream &in, Node &obj) {
    in >> 
    obj.d_idNode >>
    obj.d_idDepot >>
    obj.d_latitude >>
    obj.d_longitude >>
    obj.d_type >>
    obj.d_timeServ >>
    obj.d_qDel >>
    obj.d_qCol >>
    obj.d_twDepotStart >>
    obj.d_twDepotEnd >>
    obj.d_twCustStart >>
    obj.d_twCustEnd;

    // Check if there is another value on the line
    if (in.peek() != '\n' && in.peek() != EOF) {
        in >> obj.d_pc;
    }
    return in;
}

std::ostream &operator <<(std::ostream &out, Node &obj) {
    out <<
    obj.d_idNode << ", " <<
    obj.d_idDepot << ", " <<
    obj.d_latitude << ", " <<
    obj.d_longitude << ", " <<
    obj.d_type << ", " <<
    obj.d_timeServ << ", " <<
    obj.d_qDel << ", " <<
    obj.d_qCol << ", " <<
    obj.d_twDepotStart << ", " <<
    obj.d_twDepotEnd << ", " <<
    obj.d_twCustStart << ", " <<
    obj.d_twCustEnd;
    return out;
}

Instance::Instance() {
    /*
    Empty constructor for the instance class.
    */
    d_name = "";
    d_nVehTypes = 0;
    d_nVehTypesInfo = 0;
    d_nSL = 0;
    d_nNodes = 0;
    d_nDepots = 0;
    d_nCustNodes = 0;
    d_timeHorizonMax = 0;
}


bool Instance::load(std::string instPath) {
    /*
    Load function import all the data from the instance file.
    */
    std::string line;
    std::string headerLine;

    // Open file
    std::ifstream file;
    file.open(instPath);
    if (!file.is_open()){
        std::cout << "ERROR: Could not open file: "<< instPath << std::endl;
        return false;
    }

    // Read header
    if (!file.eof()) {
        std::getline(file, headerLine, ',');
        std::stringstream ss_header(headerLine);
        ss_header >> d_name >> d_nVehTypes >> 
        d_nVehTypesInfo >> d_nSL >> 
        d_nNodes;
    } else {
        std::cout << "ERROR: File does not contain any data. File: " << instPath << std::endl;
        return false;
    }
    
    // Read vehicle types
    std::getline(file, line); // Header line
    for (std::size_t i = 0; i < d_nVehTypes; i++) {
        if (!file.eof()) {
            std::getline(file, line);
            std::istringstream iss(line);
            VehicleType vehLine = VehicleType();
            iss >> vehLine;
            // transform speed to km/s
            vehLine.d_speed = vehLine.d_speed / 3600;
            d_vehTypes.push_back(vehLine);
        } else {
            std::cout << "ERROR: File ended earlier then expected in vehicle types. File: " << instPath << std::endl;
            std::cout << "ERROR: The number on top of the instance file does not correspond to the number of lines in the instance file.\n";
            return false;
        }
    }
    std::getline(file, line); // Empty line

    // Read vehicle types info
    std::getline(file, line); // Header line
    for (std::size_t i = 0; i < d_nVehTypesInfo; i++) {
        if (!file.eof()) {
            std::getline(file, line);
            std::istringstream iss(line);
            VehicleTypeInfo vehInfoLine = VehicleTypeInfo();
            iss >> vehInfoLine;
            // Relax if given
            d_vehTypesInfo.push_back(vehInfoLine);
        } else {
            std::cout << "ERROR: File ended earlier then expected in vehicle types INFO. File: " << instPath << std::endl;
            std::cout << "ERROR: The number on top of the instance file does not correspond to the number of lines in the instance file.\n";
            return false;
        }
    }
    std::getline(file, line); // Empty line

    // Read Scheduled lines
    std::getline(file, line); // Header line
    for (std::size_t i = 0; i < d_nSL; i++) {
        if (!file.eof()) {
            std::getline(file, line);
            std::istringstream iss(line);
            ScheduledLine scheduledLine = ScheduledLine();
            iss >> scheduledLine;
            // Add to slMap
            d_slMap[std::make_tuple(scheduledLine.d_idFrom, scheduledLine.d_idTo)].push_back(i);
            d_scheduledLines.push_back(scheduledLine);
        } else {
            std::cout << "ERROR: File ended earlier then expected in scheduled lines. File: " << instPath << std::endl;
            std::cout << "ERROR: The number on top of the instance file does not correspond to the number of lines in the instance file.\n";
            return false;
        }
    }
    std::getline(file, line); // Empty line

    // Read nodes
    std::getline(file, line); // Header line
    d_nDepots = 0;
    d_nCustNodes = 0;
    d_nodes.reserve(d_nNodes);
    d_nodeEmbeddings.reserve(d_nNodes);
    std::size_t countEmptyLines = 0;
    for (std::size_t i = 0; i < d_nNodes; i++) {
        if (!file.eof()) {
            std::getline(file, line);
            std::istringstream iss(line);
            if (line.empty()) {
                std::cout << "WARNING: Empty line in node data. File: " << instPath << std::endl;
                countEmptyLines++;
                continue;
            }
            Node node = Node();
            iss >> node;
            if (node.d_idNode != i) {
                std::cout << "WARNING: Node ID " << node.d_idNode << " does not match the line number. Setting to line number..." << instPath << std::endl;
                node.d_idNode = i;
            }
            if (node.d_pc > 9999) {
                std::cout << "WARNING: Postal code " << node.d_pc << " is larger than 9999. Setting to 0..." << instPath << std::endl;
                node.d_pc = 0;
            }
            d_nodes.push_back(node);
            d_nodeEmbeddings.push_back({static_cast<float>(node.d_latitude), static_cast<float>(node.d_longitude),
                                        static_cast<float>(node.d_qCol), static_cast<float>(node.d_qDel)});
            if (node.d_type == -1) {
                d_nDepots++;
                d_timeHorizonMax = node.d_twDepotEnd;
            } else {
                d_nCustNodes++;
            }
        } else {
            std::cout << "ERROR: File ended earlier then expected in customer data. File: " << instPath << std::endl;
            std::cout << "ERROR: The number on top of the instance file does not correspond to the number of lines in the instance file.\n";
            return false;
        }
    }
    d_nNodes = d_nNodes - countEmptyLines;
    file.close();

    // Normalize the node embeddings (init quantities at minimum of 1 to avoid division by 0)
    std::vector<float> minEmbeddings = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 0, 0};
    std::vector<float> maxEmbeddings = {std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), 1, 1};
    for (std::vector<float> &embedding : d_nodeEmbeddings) {
        for (std::size_t i = 0; i < 4; i++) {
            if (embedding[i] < minEmbeddings[i]) {
                minEmbeddings[i] = embedding[i];
            }
            if (embedding[i] > maxEmbeddings[i]) {
                maxEmbeddings[i] = embedding[i];
            }
        }
    }
    for (std::vector<float> &embedding : d_nodeEmbeddings) {
        for (std::size_t i = 0; i < 4; i++) {
            embedding[i] = (embedding[i] - minEmbeddings[i]) / (maxEmbeddings[i] - minEmbeddings[i]);
        }
    }
    return true;
}

void Instance::addAllocatedDepots(std::string tallPath) {
    /*
    Loads the depot assignment for each node based on a tall file.
    */
    // Open file
    std::ifstream file;
    file.open(tallPath);
    if (!file.is_open()){
        std::cout << "ERROR: Could not open territory allocation file at: "<< tallPath << std::endl;
        return;
    }
    // Read header
    std::string headerLine;
    if (!file.eof()) {
        std::getline(file, headerLine);
    } else {
        std::cout << "ERROR: File does not contain any data. File: " << tallPath << std::endl;
        return;
    }
    // Read depot allocation
    std::map<std::size_t, std::size_t> depotAllocation;
    std::string line;
    std::size_t pc_max = 0;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string pc_str, idDepot_str;
        std::getline(iss, pc_str, ',');
        std::getline(iss, idDepot_str, ',');
        std::size_t pc = std::stoul(pc_str);
        std::size_t idDepot = std::stoul(idDepot_str);
        depotAllocation[pc] = idDepot;
        if (pc > pc_max) {
            pc_max = pc;
        }
    }
    file.close();

    std::size_t pcPrecision = 1;
    if (pc_max > 9999) {
        std::cout << "WARNING: Postal code " << pc_max << " is larger than 9999. Setting precision to 3..." << std::endl;
        pcPrecision = 3;
    } else if (pc_max > 999) {
        pcPrecision = 4;
    } else if (pc_max > 99) {
        pcPrecision = 3;
    } else if (pc_max > 9) {
        pcPrecision = 2;
    }

    // Add the allocated depot to the nodes
    for (Node &node : d_nodes) {
        std::size_t pc = static_cast<std::size_t>(static_cast<double>(node.d_pc) / (std::pow(10, 4 - pcPrecision)));
        if (node.d_type != -1 && depotAllocation.find(pc) != depotAllocation.end()){
            node.d_idDepotAlloc = depotAllocation[pc];
        }
    }
}

std::ostream &operator <<(std::ostream &out, Instance &obj) {
    out << "Instance: " << obj.d_name << "\n\n";

    for (VehicleType vehType : obj.d_vehTypes) {
        out << vehType << "\n";
    }
    out << "\n";

    for (VehicleTypeInfo vehTypeInfo : obj.d_vehTypesInfo) {
        out << vehTypeInfo << "\n";
    }
    out << "\n";

    for (ScheduledLine sl : obj.d_scheduledLines) {
        out << sl << "\n";
    }
    out << "\n";
    
    std::size_t iLastNode = obj.d_nodes.size() - 1;
    out << "First node loaded: " << obj.d_nodes[0] << "\n";
    out << "Last node loaded: " << obj.d_nodes[iLastNode] << "\n";
    return out;
}


