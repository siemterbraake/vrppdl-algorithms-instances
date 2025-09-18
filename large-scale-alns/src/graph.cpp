#include "graph.h"

void Graph::load(const Instance &inst, Settings &settings) {
    /*
    Graph contains distances and time between nodes
    */    
    // set the size
    d_distMatrix.resize(inst.d_nVehTypes*inst.d_nNodes*inst.d_nNodes);
    d_timeMatrix.resize(inst.d_nVehTypes*inst.d_nNodes*inst.d_nNodes);
    d_nearestNodes.resize(inst.d_nNodes*settings.d_nNeighborConsider);
    d_nNodes = inst.d_nNodes;
    d_nVehicleTypes = inst.d_nVehTypes;
    d_nNearestNodes = settings.d_nNeighborConsider;
    d_maxDist = 0;
    d_maxTime = 0;
    // Setup values
    calcDima(inst, settings);
    calcNearestNodes(inst, settings);

    std::cout << "Graph loaded succesfully\n";
}

void Graph::calcDima(const Instance &inst, const Settings &settings) {
    /*
    Calculates the distance matrix, given the distance metric
    */
    if (settings.d_distMetric == "matrix") {
        loadMatrix();
        std::cout << "Matrix loaded succesfully\n";
        return;
    }
    // Calculate distance between nodes
    for (std::size_t i = 0; i < inst.d_nNodes; i++) {
        if (inst.d_nodes[i].d_twDepotEnd > d_maxTime) {
            d_maxTime = inst.d_nodes[i].d_twDepotEnd;
        }
        for (std::size_t j = i+1; j < inst.d_nNodes; j++) {
            // Get distance
            double lat1 = inst.d_nodes[i].d_latitude;
            double lon1 = inst.d_nodes[i].d_longitude;
            double lat2 = inst.d_nodes[j].d_latitude;
            double lon2 = inst.d_nodes[j].d_longitude;

            float dist;
            if (settings.d_distMetric == "euclidean") {
                dist = calcEuclideanDist(lat1, lon1, lat2, lon2);
            } 
            else if (settings.d_distMetric == "haversine") {
                float distHav = calcHaversineDist(lat1, lon1, lat2, lon2);
                dist = distHav / 1000;
            }
            else {
                std::cerr << "ERROR: Distance metric not recognized, please use euclidean, haversine or matrix.\n";
                exit(1);
            }
            if (dist > d_maxDist) {
                d_maxDist = dist;
            }   
            for (std::size_t k = 0; k < inst.d_nVehTypes; k++) {
                setTime(k,i,j,static_cast<int>(round(dist / inst.d_vehTypes[k].d_speed)));
                setTime(k,j,i,static_cast<int>(round(dist / inst.d_vehTypes[k].d_speed)));
                setDist(k,i,j,dist);
                setDist(k,j,i,dist);
            }
        }
    }
}

void Graph::calcNearestNodes(const Instance &inst, Settings &settings) {
    /*
    Pre-calculates the list of near nodes, for the localized search
    and dynamic removal of nodes
    */
    // If the considered neighbor nodes is larger than the number of nodes, set it to the number of nodes
    if (settings.d_nNeighborConsider + 1 > inst.d_nCustNodes) {
        settings.d_nNeighborConsider = inst.d_nCustNodes - 1;
        std::cout << "WARNING: The number of considered neighbor nodes is larger than the number of customer nodes, setting it to " << settings.d_nNeighborConsider-1 << ".\n";
    }

    // Calculate nearest nodes
    float w1 = 1/d_maxDist; // Normalize distance by the maximum distance
    float w2 = 1.0f / d_maxTime; // Normalize time by the maximum time (8AM to 6PM)
    for (std::size_t i = inst.d_nDepots; i < inst.d_nNodes; i++) {
        std::vector<std::size_t> idx(inst.d_nCustNodes);
        std::iota(idx.begin(), idx.end(), inst.d_nDepots);
        std::sort(idx.begin(), idx.end(), [&](std::size_t i1, std::size_t i2) {
            return calcSpacioTemporalDist(inst.d_nodes[i], inst.d_nodes[i1], w1, w2) < 
            calcSpacioTemporalDist(inst.d_nodes[i], inst.d_nodes[i2], w1, w2);
        });
        setNeighbors(i,std::vector<std::size_t>(idx.begin()+1, idx.begin()+static_cast<int>(settings.d_nNeighborConsider)+1));
    }
}

void Graph::loadMatrix() {
    /*
    Loads distance and time matrices from csv files for each vehicle type
    */
    // Chech which path to work with
    std::string path;
    if (std::filesystem::exists("cpp-metaheuristic")) {
        path = "cpp-metaheuristic/input/";
    }
    else if (std::filesystem::exists("input")) {
        path = "input/";
    }
    else {
        std::cout << "ERROR: No input folder found. Please add an input folder to the cpp-metaheuristic directory.\n";
    }
    for (std::size_t i = 0; i < d_nVehicleTypes; i++) {
        std::string pathDima = path + "dima_" + std::to_string(i) + ".csv";
        std::string pathTima = path + "tima_" + std::to_string(i) + ".csv";
        // Check if the file exists
        std::ifstream fileDima(pathDima);
        if (fileDima.fail()) {
            std::cout << "ERROR: dima_" << i << ".csv does not exist. Please add this file to the input folder or don't use matrix as the distance metric" << std::endl;
            exit(1);
        }
        std::ifstream fileTima(pathTima);
        if (fileTima.fail()) {
            std::cout << "ERROR: tima_" << i << ".csv does not exist. Please add this file to the input folder or don't use matrix as the distance metric" << std::endl;
            exit(1);
        }

        // Load the distance matrix
        std::string line;
        std::string val;
        for (std::size_t j = 0; j < d_nNodes; j++) {
            if (fileDima.eof()) {
                std::cout << "ERROR: Too few rows in dima_" << i << ".csv.\n";
                exit(1);
            }
            std::getline(fileDima, line);
            std::stringstream ss(line);
            for (std::size_t k = 0; k < j + 1; k++) {
                std::getline(ss, val, ',');
                if (val.empty()) {
                    std::cout << "ERROR: Empty value in dima_" << i << ".csv at row " << j << " and column " << k << ".\n";
                    exit(1);
                }
                // Ensure symmetry, and more efficient
                float dist = std::stof(val);
                setDist(i,j,k,dist);
                setDist(i,k,j,dist);
                if (dist > d_maxDist && dist < 10000) {
                    d_maxDist = dist;
                }
            }    
        }
        std::getline(fileDima, line);
        if (!fileDima.eof()) {
            std::cout << "ERROR: Too many rows in dima_" << i << ".csv.\n";
            exit(1);
        }
        fileDima.close();
        // Load the time matrix
        for (std::size_t j = 0; j < d_nNodes; j++) {
            if (fileTima.eof()) {
                std::cout << "ERROR: Too few rows in dima_" << i << ".csv.\n";
                exit(1);
            }
            std::getline(fileTima, line);
            std::stringstream ss(line);
            for (std::size_t k = 0; k < j + 1; k++) {
                std::getline(ss, val, ',');
                if (val.empty()) {
                    std::cout << "ERROR: Empty value in tima_" << i << ".csv at row " << j << " and column " << k << ".\n";
                    exit(1);
                }
                // Ensure symmetry, and more efficient
                setTime(i,j,k,std::stoi(val));
                setTime(i,k,j,std::stoi(val));
            }
        }
        std::getline(fileTima, line);
        if (!fileTima.eof()) {
            std::cout << "ERROR: Too many rows in tima_" << i << ".csv.\n";
            exit(1);
        }
        fileTima.close();
    }
}

float Graph::calcSpacioTemporalDist(const Node &n1, const Node &n2, float w1, float w2) const {
    /*
    Calculate the spacio-temporal distance between two nodes
    */
    // Set node i equal to the node with the earliest customer time window start
    const Node *i = &n1;
    const Node *j = &n2;
    if (n1.d_twCustStart > n2.d_twCustStart) {
        i = &n2;
        j = &n1;
    }
    float distT = 0;
    if (i->d_twCustEnd < j->d_twCustStart) {
        distT = static_cast<float>(j->d_twCustStart - i->d_twCustEnd) * w2;
    }
    return getDist(0,i->d_idNode,j->d_idNode) * w1 + distT;
}

float Graph::calcHaversineDist(double lat1, double lon1, double lat2, double lon2) const {
    /*
    Calculate the haversine distance between two points (lat1, lon1) and (lat2, lon2)
    */ 
    const double p = 0.017453292519943295;  // Math.PI / 180
    float a = static_cast<float>(0.5f - cos((lat2 - lat1) * p) / 2 +
        cos(lat1 * p) * cos(lat2 * p) * 
        (1 - cos((lon2 - lon1) * p)) / 2);
    return 12742000 * asin(sqrt(a));        // 2 * R where R = 6,371,000 meters
}

float Graph::calcEuclideanDist(double x1, double y1, double x2, double y2) const {
    /*
    Calculate the euclidean distance between two points (x1, y1) and (x2, y2)
    */
    return static_cast<float>(sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)));
}

float Graph::getDist(std::size_t idVehicleType, std::size_t idNode1, std::size_t idNode2) const {
    return d_distMatrix[d_nNodes * d_nNodes * idVehicleType + d_nNodes * idNode1 + idNode2];
}

int Graph::getTime(std::size_t idVehicleType, std::size_t idNode1, std::size_t idNode2) const {
    return d_timeMatrix[d_nNodes * d_nNodes * idVehicleType + d_nNodes * idNode1 + idNode2];
}

std::pair<std::vector<size_t>::const_iterator, std::vector<size_t>::const_iterator> Graph::getNearestNodes(std::size_t idNode) const {
    return {d_nearestNodes.cbegin() + static_cast<int>(idNode * d_nNearestNodes), d_nearestNodes.cbegin() + static_cast<int>((idNode + 1) * d_nNearestNodes)};
}

void Graph::setDist(std::size_t idVehicleType, std::size_t idNode1, std::size_t idNode2, float dist) {
    d_distMatrix[d_nNodes * d_nNodes * idVehicleType + d_nNodes * idNode1 + idNode2] = dist;
}

void Graph::setTime(std::size_t idVehicleType, std::size_t idNode1, std::size_t idNode2, int time) {
    d_timeMatrix[d_nNodes * d_nNodes * idVehicleType + d_nNodes * idNode1 + idNode2] = time;
}

void Graph::setNeighbors(std::size_t idNode, std::vector<std::size_t> nearestNodes) {
    std::copy(nearestNodes.begin(), nearestNodes.end(), d_nearestNodes.begin() + static_cast<int>(idNode * d_nNearestNodes));
}
