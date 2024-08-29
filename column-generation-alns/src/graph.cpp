#include "graph.hpp"

void Graph::load(const Instance &inst, Settings &settings) {
    /*
    Graph contains distances and time between nodes
    */    
    // set the size
    d_distMatrix.resize(inst.d_nVehTypes * inst.d_nNodes * inst.d_nNodes);
    d_timeMatrix.resize(inst.d_nVehTypes * inst.d_nNodes * inst.d_nNodes);
    d_nearestNodes.resize(inst.d_nNodes * settings.d_maxNeighborhoodSize);
    d_nNodes = inst.d_nNodes;
    d_nVehicleTypes = inst.d_nVehTypes;
    d_nNearestNodes = settings.d_maxNeighborhoodSize;
    d_maxDist = 0;
    // Setup values
    calcDima(inst, settings);
    calcNearestNodes(inst, settings);

    std::cout << "Graph loaded succesfully\n";
}

void Graph::calcDima(const Instance &inst, const Settings &settings) {
    /*
    Calculates the distance matrix, given the distance metric
    */
    for (unsigned short i = 0; i < inst.d_nNodes; i++) {
        for (unsigned short j = i+1; j < inst.d_nNodes; j++) {
            // Get distance
            double lat1 = inst.d_nodes[i].d_latitude;
            double lon1 = inst.d_nodes[i].d_longitude;
            double lat2 = inst.d_nodes[j].d_latitude;
            double lon2 = inst.d_nodes[j].d_longitude;

            double dist;
            if (settings.d_distMetric == "euclidean") {
                dist = calcEuclideanDist(lat1, lon1, lat2, lon2);
            } 
            else if (settings.d_distMetric == "haversine") {
                double distHav = calcHaversineDist(lat1, lon1, lat2, lon2);
                dist = distHav / 1000;
            }
            else {
                std::cerr << "ERROR: Distance metric not recognized, please use euclidean, haversine or matrix.\n";
                exit(1);
            }
            if (dist > d_maxDist) {
                d_maxDist = dist;
            }   
            for (unsigned short k = 0; k < inst.d_nVehTypes; k++) {
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
    if (settings.d_maxNeighborhoodSize + 1 > inst.d_nCustNodes) {
        settings.d_maxNeighborhoodSize = inst.d_nCustNodes - 1;
        if (settings.d_verbose) {
            std::cout << "WARNING: The number of considered neighbor nodes is larger than the number of customer nodes, setting it to " << settings.d_maxNeighborhoodSize-1 << ".\n";
        }
    }

    // Calculate nearest nodes
    double w1 = 1/d_maxDist; // Normalize distance by the maximum distance
    double w2 = 1.0 / (10*60*60); // Normalize time by the maximum time (8AM to 6PM)
    for (unsigned short i = inst.d_nDepots; i < inst.d_nNodes; i++) {
        std::vector<unsigned short> idx(inst.d_nCustNodes);
        std::iota(idx.begin(), idx.end(), inst.d_nDepots);
        std::sort(idx.begin(), idx.end(), [&](unsigned short i1, unsigned short i2) {
            return calcSpacioTemporalDist(inst.d_nodes[i], inst.d_nodes[i1], w1, w2) < 
            calcSpacioTemporalDist(inst.d_nodes[i], inst.d_nodes[i2], w1, w2);
        });
        setNeighbors(i,std::vector<unsigned short>(idx.begin()+1, idx.begin()+static_cast<int>(settings.d_maxNeighborhoodSize)+1));
    }
}

double Graph::calcSpacioTemporalDist(const Node &n1, const Node &n2, double w1, double w2) const {
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
    double distT = 0;
    if (i->d_twCustEnd < j->d_twCustStart) {
        distT = static_cast<double>(j->d_twCustStart - i->d_twCustEnd) * w2;
    }
    return getDist(0,i->d_idNode,j->d_idNode) * w1 + distT;
}

double Graph::calcHaversineDist(double lat1, double lon1, double lat2, double lon2) const {
    /*
    Calculate the haversine distance between two points (lat1, lon1) and (lat2, lon2)
    */ 
    const double p = 0.017453292519943295;  // Math.PI / 180
    double a = static_cast<double>(0.5f - cos((lat2 - lat1) * p) / 2 +
        cos(lat1 * p) * cos(lat2 * p) * 
        (1 - cos((lon2 - lon1) * p)) / 2);
    return 12742000 * asin(sqrt(a));        // 2 * R where R = 6,371,000 meters
}

double Graph::calcEuclideanDist(double x1, double y1, double x2, double y2) const {
    /*
    Calculate the euclidean distance between two points (x1, y1) and (x2, y2)
    */
    return static_cast<double>(sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)));
}

void Graph::setDist(unsigned short idVehicleType, unsigned short idNode1, unsigned short idNode2, double dist) {
    d_distMatrix[d_nNodes * d_nNodes * idVehicleType + d_nNodes * idNode1 + idNode2] = dist;
}

void Graph::setTime(unsigned short idVehicleType, unsigned short idNode1, unsigned short idNode2, int time) {
    d_timeMatrix[d_nNodes * d_nNodes * idVehicleType + d_nNodes * idNode1 + idNode2] = time;
}

void Graph::setNeighbors(unsigned short idNode, std::vector<unsigned short> nearestNodes) {
    std::copy(nearestNodes.begin(), nearestNodes.end(), d_nearestNodes.begin() + static_cast<int>(idNode * d_nNearestNodes));
}


