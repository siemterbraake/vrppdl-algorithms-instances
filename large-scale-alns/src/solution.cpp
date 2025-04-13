#include "solution.h"

Solution::Solution() {
    /*
    Empty constructor for the solution struct.
    */
    d_cost = 0;
    d_pInst = nullptr;
    d_pGraph = nullptr;
    d_pSettings = nullptr;
}

Solution::Solution(const Instance& inst, const Graph& graph, const Settings& settings) {
    /*
    Constructor for the solution struct.
    */
    // Set sizes for arrays
    d_depotSols.resize(inst.d_nDepots);
    d_SLSols.resize(inst.d_nSL);
    d_reqUnassigned.resize(inst.d_nCustNodes);

    // Create SLSolutions
    for (std::size_t i = 0; i < inst.d_nSL; i++) {
        d_SLSols[i] = SLSolution(i);
    }

    // Create depot solutions
    for (std::size_t i = 0; i < inst.d_nDepots; i++) {
        d_depotSols[i] = DepotSolution(i, inst.d_vehTypesInfo);
    }

    // Set all requests to unassigned
    std::size_t idCustNode = inst.d_nDepots;
    for (std::size_t i = 0; i < inst.d_nCustNodes; i++) {
        d_reqUnassigned[i] = idCustNode;
        idCustNode++;
    }
    // Set cost to zero
    d_cost = 0;

    // Set pointers to the instance and graph
    d_pInst = &inst;
    d_pGraph = &graph;
    d_pSettings = &settings;
}

void Solution::insertNode(const Node &req, std::size_t pos, std::size_t idDepot, 
                               std::size_t idVehicleType, std::size_t idRoute, std::size_t idSL,
                               int twDepotStart, int twDepotEnd, float costInsert) {
    /*
    Performs the necessary actions to insert a request into the solution
    - When a route is found, insert the request into the route
    - When no route is found, open a new route in a Greedy manner
    - When no vehicle is available, set the solution to the current solution
    - Manage the cost and visitLocs
    */ 
    // Remove the request from the unassigned list
    if (idDepot == std::numeric_limits<std::size_t>::max()){
        if (d_pSettings->d_verbose) {    
            std::cerr << "No vehicle available for request " << req.d_idNode << " and no previous solution available, adding to vector of infeasible requests" << std::endl;
        }
        d_reqInfeasible.push_back(req.d_idNode);
        d_reqUnassigned.erase(d_reqUnassigned.begin());
        return;
    }
    d_reqUnassigned.erase(d_reqUnassigned.begin());
    if (idRoute == std::numeric_limits<std::size_t>::max()) {
        // Open a new route
        d_depotSols[idDepot].d_vehTypeSols[idVehicleType].openRoute(req,twDepotStart, twDepotEnd, d_pSettings->d_excessiveRoutePenalty,
                                                                        d_pInst->d_nodes[idDepot], *d_pGraph, d_pInst->d_vehTypes[idVehicleType]);
        idRoute = d_depotSols[idDepot].d_vehTypeSols[idVehicleType].d_nUsedVehicles-1;
        pos = 1;
    }
    else {
        // Otherwise insert at the id position
        d_depotSols[idDepot].d_vehTypeSols[idVehicleType].d_routes[idRoute].insert(req,pos,twDepotStart, twDepotEnd,*d_pGraph,
                                                                                   d_pInst->d_vehTypes[idVehicleType], d_visitLocs);
    }
    std::size_t posSL;
    if (idSL != std::numeric_limits<std::size_t>::max()) {
        posSL = d_SLSols[idSL].d_nAssigned;
        d_SLSols[idSL].assign(req);
    }
    else {
        posSL = std::numeric_limits<std::size_t>::max();
    }
    d_cost += costInsert;
    d_visitLocs[req.d_idNode] = VisitLoc(idDepot,idSL,posSL,idVehicleType,idRoute,pos);
}

void Solution::checkOpenRouteGreedy(const Node &req, bool noise, bool restrictServiceArea, std::size_t &bestDepot, std::size_t &bestVehicle, 
                                    int &bestTwDepotStart, int &bestTwDepotEnd, std::size_t &bestSL, float &costOpen, std::mt19937 &gen) {
    /*
    Checks what the cheapest depot is to open a new route in a Greedy manner
    */
    // Iterate over each depot
    std::vector<std::size_t> idDepots;
    if (restrictServiceArea) {
        idDepots = {req.d_idDepotAlloc};
    }
    else {
        idDepots = std::vector<std::size_t>(d_pInst->d_nDepots);
        std::iota(idDepots.begin(), idDepots.end(), static_cast<std::size_t>(0));
    }
    for (std::size_t i = 0; i < idDepots.size(); i++) {
        // Set helper variables for the scheduled line selection
        bool found = false;
        std::size_t idDepot = idDepots[i];
        std::size_t SL = std::numeric_limits<std::size_t>::max();
        int twDepotStart = req.d_twDepotStart;
        int twDepotEnd = req.d_twDepotEnd;
        // Check if the request needs a scheduled line
        if (idDepot!=req.d_idDepot) {
            selectSL(SL,found,twDepotStart,twDepotEnd,req,idDepot);
            // If no scheduled line is found, skip the depot
            if (!found) {
                continue;
            }
        }
        // Iterate over each vehicle type
        for (std::size_t j = 0; j < d_pInst->d_nVehTypes; j++) {
            float costOpenTemp = d_depotSols[idDepot].d_vehTypeSols[j].checkOpen(req,twDepotStart,twDepotEnd, d_pSettings->d_excessiveRoutePenalty,
                                                                                     *d_pGraph, d_pInst->d_vehTypes[j]);
            // Check if the costOpenTemp is std::numeric_limits<float>::max(), if so, skip the vehicle type
            if (costOpenTemp == std::numeric_limits<float>::max()) {
                continue;
            }
            // If a scheduled line is used, add the cost of the scheduled line
            if (SL != std::numeric_limits<std::size_t>::max()) {
                costOpenTemp += d_pInst->d_scheduledLines[SL].d_costRequest*static_cast<float>(req.d_qCol+req.d_qDel);
            }
            // Add noise if necessary
            float costNoise = 0;
            if (noise) {
                costNoise = std::uniform_real_distribution<float>(-1,1)(gen)*d_pSettings->d_noiseLevel*costOpenTemp;
            }
            if (costOpenTemp < costOpen + costNoise) {
                bestDepot = idDepot;
                bestVehicle = j;
                bestTwDepotStart = twDepotStart;
                bestTwDepotEnd = twDepotEnd;
                costOpen = costOpenTemp;
                bestSL = SL;
            }
        }
    }
}

void Solution::removeNode(std::size_t idNode) {
    /* 
    Removes a node from the solution
    */
    if (d_visitLocs.find(idNode) == d_visitLocs.end()) {
        return;
    }
    const VisitLoc &loc = d_visitLocs.at(idNode);
    const Node &req = d_pInst->d_nodes[idNode];
    VehicleTypeSolution &vehTypeSol = d_depotSols[loc.d_idDepot].d_vehTypeSols[loc.d_idVehicleType];
    Route &route = vehTypeSol.d_routes[loc.d_idRoute];

    // If there is one customer in the route, remove the route
    if (route.d_nVisits == 3) {
        std::size_t ctrRemoved = 0;
        removeRoute(loc.d_idDepot,loc.d_idVehicleType,loc.d_idRoute, ctrRemoved);
        return;
    }

    // Perform removal
    float costBefore = route.d_cost;
    route.remove(req, loc.d_idVisit, *d_pGraph, d_pInst->d_vehTypes[loc.d_idVehicleType], *d_pInst, d_visitLocs);
    float costAfter = route.d_cost;
    // Remove the node from the SL
    float costSL = 0;
    if (loc.d_idSL != std::numeric_limits<std::size_t>::max()) {
        d_SLSols[loc.d_idSL].remove(req, d_visitLocs);
        costSL = d_pInst->d_scheduledLines[loc.d_idSL].d_costRequest*static_cast<float>(req.d_qCol+req.d_qDel);
    }
    // Manage bookkeeping of node locations
    d_reqUnassigned.push_back(idNode);
    d_visitLocs.erase(idNode);
    // Update the solution cost
    d_cost -= (costBefore - costAfter) + costSL;

    // If the route ends up too long, remove the route
    if (route.d_timeTravel > d_pInst->d_vehTypes[vehTypeSol.d_idVehicleType].d_timeMax) {
        std::size_t ctrRemoved = 0;
        removeRoute(route.d_idDepot,route.d_idVehicleType,route.d_idRoute, ctrRemoved);
    }
}

void Solution::removeRoute(std::size_t idDepot, std::size_t idVehicleType, std::size_t idRoute, std::size_t &ctrRemoved) {
    /*
    Removes a route from the solution
    */
    // Check if the vehicle type is overused
    float costPenalty = 0;
    VehicleTypeSolution &vehTypeSol = d_depotSols[idDepot].d_vehTypeSols[idVehicleType];
    Route &route = vehTypeSol.d_routes[idRoute];
    if (vehTypeSol.d_nUsedVehicles > vehTypeSol.d_nVehicles) {
        costPenalty = d_pSettings->d_excessiveRoutePenalty;
    }
    float costSLRoute = 0;
    for (std::size_t i = 1; i < route.d_nVisits-1; i++) {
        std::size_t idNode = route.d_visits[i].d_idNode;
        // Add to the unassigned list
        d_reqUnassigned.push_back(idNode);
        // Remove from SL
        float costSL = 0;
        if (d_visitLocs.at(idNode).d_idSL != std::numeric_limits<std::size_t>::max()) {
            const Node req = d_pInst->d_nodes[idNode];
            costSL = d_pInst->d_scheduledLines[d_visitLocs.at(idNode).d_idSL].d_costRequest*static_cast<float>(req.d_qCol+req.d_qDel);
            d_SLSols[d_visitLocs.at(idNode).d_idSL].remove(req, d_visitLocs);
        }
        // Remove from visitLocs
        d_visitLocs.erase(idNode);
        costSLRoute += costSL;
        ctrRemoved++;
    }
    // Remove the route
    float costRoute = route.d_cost;
    vehTypeSol.closeRoute(idRoute, d_pSettings->d_excessiveRoutePenalty, d_visitLocs);
    // Update the solution cost
    d_cost -= (costRoute + costSLRoute + costPenalty);
    // Update the neighborhood size
}

void Solution::selectSL(std::size_t &SL, bool &found, int &twDepotStart, 
                        int &twDepotEnd, const Node &req, std::size_t idDepot) const {
    /*
    Selects a scheduled line for a request given the depot and the request
    */
    std::size_t idSL;
    if (req.d_type == 0) {
        // Case of a collection request
        // If the depot id is higher than the number of depots, the depot assignment is free.
        // If it was scheduled on a line on the same day, the fixed time for the given depot is used
        if (req.d_idDepot >= d_pInst->d_nDepots) {
            found = true;
            if (req.d_twDepotEnd < d_pInst->d_nodes[idDepot].d_twDepotEnd) {
                // SL Deadline for the given depot is stored in the customer tw end
                twDepotEnd = d_pInst->d_nodes[idDepot].d_twCustEnd;
            }
            return;
        }
        // Select the latest possible scheduled line
        const std::vector<std::size_t> sls = d_pInst->d_slMap.at(std::make_tuple(static_cast<std::size_t>(idDepot),req.d_idDepot));
        idSL = sls.size()-1;
        while (!found && idSL != std::numeric_limits<std::size_t>::max()) {
            if (d_SLSols[sls[idSL]].d_qAssigned + req.d_qCol <= d_pInst->d_scheduledLines[sls[idSL]].d_capacity &&
                d_pInst->d_scheduledLines[sls[idSL]].d_timeArr <= req.d_twDepotEnd) {
                SL = sls[idSL];
                found = true;
                twDepotEnd = d_pInst->d_scheduledLines[SL].d_timeDep;
            }
            idSL--;
        }
    }
    else if (req.d_type == 1) {
        // Case of a delivery request
        // Select the earliest possible scheduled line
        const std::vector<std::size_t> sls = d_pInst->d_slMap.at(std::make_tuple(req.d_idDepot,static_cast<std::size_t>(idDepot)));
        idSL = 0;
        while (!found && idSL < sls.size()) {
            if (d_SLSols[sls[idSL]].d_qAssigned + req.d_qDel <= d_pInst->d_scheduledLines[sls[idSL]].d_capacity &&
                d_pInst->d_scheduledLines[sls[idSL]].d_timeDep >= req.d_twDepotStart) {
                SL = sls[idSL];
                found = true;
                twDepotStart = d_pInst->d_scheduledLines[SL].d_timeArr;
            }
            idSL++;
        }
    }
}

void Solution::patchCosts() {
    /*
    Patch the costs of the solution after the destroy and repair operators.
    */
    float costDist = 0; // Cost based on the sum of all route distances
    float costSL = 0; // Cost based on the sum of all scheduled line costs
    float costPenalty = 0; // Cost based on the sum of all penalties
    float tol = 0.9f; // Tolerance for floating point comparison
    
    // Iterate over scheduled lines
    for (std::size_t i = 0; i < d_pInst->d_nSL; i++) {
        // Update total cost of scheduled lines
        costSL += static_cast<float>(d_SLSols[i].d_qAssigned)*d_pInst->d_scheduledLines[i].d_costRequest;
    }
    // Iterate over depot solutions
    for (std::size_t i = 0; i < d_pInst->d_nDepots; i++) {
        // Iterate over vehicle type solutions
        for (std::size_t j = 0; j < d_depotSols[i].d_nVehicleTypes; j++) {
            float costVeh = 0;
            VehicleTypeSolution &curVehTypeSol = d_depotSols[i].d_vehTypeSols[j];
            costPenalty += curVehTypeSol.d_penalty;
            // Iterate over routes
            for (std::size_t k = 0; k < curVehTypeSol.d_nUsedVehicles; k++) {
                float routeDist = 0;
                // Create pointer to the current route
                Route& curRoute = curVehTypeSol.d_routes[k];
                // Iterate over visits
                for (std::size_t l = 1; l < curRoute.d_nVisits; l++) {
                    // Create pointer to the current visit
                    Visit& curVisit = curRoute.d_visits[l];
                    Visit& prevVisit = curRoute.d_visits[l-1];
                    // Check if the request is also assigned to a scheduled line if necessary
                    routeDist += d_pGraph->getDist(curVehTypeSol.d_idVehicleType,prevVisit.d_idNode,curVisit.d_idNode);     
                }
                // Verify that the distance-based calculation is equal to the cost-based calculation
                float costRoute = routeDist*d_pInst->d_vehTypes[j].d_costVariable+d_pInst->d_vehTypes[j].d_costFixed;
                if (std::abs(costRoute - curRoute.d_cost) > tol) {
                    std::cout << "WARNING: Distance-based calculation is not equal to cost-based calculation for route" << k << " of vehicle type " << j << ".\n";
                }
                curRoute.d_cost = costRoute;
                costVeh += costRoute;
            }
            // Update distance-based cost for the vehicle type solution
            costDist += costVeh;
        }
    }
    // Verify that the distance-based calculation is equal to the cost-based calculation
    if (std::abs(costDist + costSL + costPenalty - d_cost) > tol) {
        std::cout << "WARNING: Distance-based calculation is not equal to cost-based calculation\n";
    }
    // Set equal to prevent drift
    d_cost = costDist + costSL + costPenalty;
}

void Solution::sortRequests(std::size_t logic) {
    /*
    Sorts the requests based on one of three predefined sets of logic
    #1: Type first, then capacity
    #2: Type first, then time window
    #3: Type first, then distance to depot
    #4: First distance to depot, then type
    */
    // Logic 1: Type first, then capacity
    switch (logic) {
        case 1:
            std::sort(d_reqUnassigned.begin(), d_reqUnassigned.end(), [&](std::size_t a, std::size_t b) {
                if (d_pInst->d_nodes[a].d_type != d_pInst->d_nodes[b].d_type) {
                    return d_pInst->d_nodes[a].d_type < d_pInst->d_nodes[b].d_type;
                }
                else {
                    return d_pInst->d_nodes[a].d_qDel + d_pInst->d_nodes[a].d_qCol < d_pInst->d_nodes[b].d_qDel + d_pInst->d_nodes[b].d_qCol;
                }
            });
            break;
        case 2:
            std::sort(d_reqUnassigned.begin(), d_reqUnassigned.end(), [&](std::size_t a, std::size_t b) {
                if (d_pInst->d_nodes[a].d_type != d_pInst->d_nodes[b].d_type) {
                    return d_pInst->d_nodes[a].d_type < d_pInst->d_nodes[b].d_type;
                }
                else {
                    return d_pInst->d_nodes[a].d_twCustStart > d_pInst->d_nodes[b].d_twCustStart;
                }
            });
            break;
        case 3:
            std::sort(d_reqUnassigned.begin(), d_reqUnassigned.end(), [&](std::size_t a, std::size_t b) {
                if (d_pInst->d_nodes[a].d_type != d_pInst->d_nodes[b].d_type) {
                    return d_pInst->d_nodes[a].d_type < d_pInst->d_nodes[b].d_type;
                }
                else {
                    return d_pGraph->getDist(0,d_pInst->d_nodes[a].d_idDepot,a) < d_pGraph->getDist(0,d_pInst->d_nodes[b].d_idDepot,b);
                }
            });
            break;
        case 4:
            std::sort(d_reqUnassigned.begin(), d_reqUnassigned.end(), [&](std::size_t a, std::size_t b) {
                if (d_pGraph->getDist(0,d_pInst->d_nodes[a].d_idDepot,a) != d_pGraph->getDist(0,d_pInst->d_nodes[b].d_idDepot,b)) {
                    return d_pGraph->getDist(0,d_pInst->d_nodes[a].d_idDepot,a) < d_pGraph->getDist(0,d_pInst->d_nodes[b].d_idDepot,b);
                }
                else {
                    return d_pInst->d_nodes[a].d_type < d_pInst->d_nodes[b].d_type;
                }
            });
            break;
        default:
            std::cout << "ERROR: Sorting logic not implemented. Returning without sorting\n";
            break;
    }
}


void Solution::updateSubproblems() {
    /*
    Update the list of subproblems
    */
    // Clear the subproblems
    std::size_t nRoutes = 0;
    for (std::size_t i = 0; i < d_depotSols.size(); i++) {
        for (std::size_t j = 0; j < d_depotSols[i].d_nVehicleTypes; j++) {
            nRoutes += d_depotSols[i].d_vehTypeSols[j].d_nUsedVehicles;
        }
    }
    d_subproblems.resize(nRoutes);
    // For each route, take the anchor node as the most central node and construct
    // the subproblem based on the dynamic removal logic
    size_t idSubproblem = 0;
    for (std::size_t i = 0; i < d_depotSols.size(); i++) {
        for (std::size_t j = 0; j < d_depotSols[i].d_nVehicleTypes; j++) {
            for (std::size_t k = 0; k < d_depotSols[i].d_vehTypeSols[j].d_nUsedVehicles; k++) {
                const Route &route = d_depotSols[i].d_vehTypeSols[j].d_routes[k];
                d_subproblems[idSubproblem] = constructSubproblem(route, d_pSettings->d_subproblemSize);
                idSubproblem++;
            }
        }
    }
}

std::vector<std::size_t> Solution::constructSubproblem(const Route &route, std::size_t size) const {
    /*
    Creates the subproblem for a given route, based on the dynamic removal logic
    */
    float routeCoef = 0.5f;
    std::size_t nAddRoute = std::min((std::size_t)std::round(static_cast<float>(size)*routeCoef), route.d_nVisits-2);
    std::size_t ctrAdded = 0;
    std::vector<std::size_t> subproblem(size);
    // Determine the anchor node as the central node of the route
    std::size_t idVisit = static_cast<std::size_t>(route.d_nVisits / 2);
    std::size_t idAnchor = route.d_visits[idVisit].d_idNode;
    if (nAddRoute == route.d_nVisits-2) {
        // add the entire route
        for (std::size_t i = 1; i < route.d_nVisits-1; i++) {
            subproblem[ctrAdded] = route.d_visits[i].d_idNode;
            ctrAdded++;
        }
    }
    else {
        while (ctrAdded < nAddRoute) {
            // add adjacent nodes to the central node until nRemRoute is reached
            std::size_t idNode= route.d_visits[idVisit].d_idNode;
            subproblem[ctrAdded] = idNode;
            ctrAdded++;
            // Check if crtRemoved is divisible by 2, and the limits are not exceeded
            if (ctrAdded % 2 == 0 && idVisit > 1) {
                idVisit--;
            }
            if (idVisit >= route.d_nVisits-1) {
                idVisit--;
            }
        }
    }
    // Perform nn-remove on the remaining nodes
    auto nearestNodes = d_pGraph->getNearestNodes(idAnchor);
    while (ctrAdded < size) {
        std::size_t idNode = *nearestNodes.first;
        // Check if the node is in visitLocs
        auto it = std::find(subproblem.begin(), subproblem.end(), idNode);
        if (it == subproblem.end()) {
            // Add the node to the subproblem
            subproblem[ctrAdded] = idNode;
            ctrAdded++;
        }
        ++nearestNodes.first;
    }
    return subproblem;
}

float Solution::getPenalty() {
    /*
    Calculates the penalty for the solution
    */
    float penalty = 0;
    for (std::size_t i = 0; i < d_depotSols.size(); i++) {
        for (std::size_t j = 0; j < d_depotSols[i].d_nVehicleTypes; j++) {
            penalty += d_depotSols[i].d_vehTypeSols[j].d_penalty;
        }
    }
    return penalty;
}

void Solution::writeSLUsage(std::ofstream &file) const {
    /*
    Prints the usage of the scheduled lines in the solution
    */
    for (std::size_t i = 0; i < d_SLSols.size(); i++) {
        float perc = 100*static_cast<float>(d_SLSols[i].d_qAssigned)/static_cast<float>(d_pInst->d_scheduledLines[i].d_capacity);
        file << "SL " << i << " is utilized at " << perc << "% by " << d_SLSols[i].d_nAssigned << " shipments\n";
    }
}

void Solution::writeCsv(std::string path) const {
    /*
    Writes the solution to a csv file.
    */
    std::cout << "Writing solution to csv file...\n";
    std::ofstream file;
    file.open(path);
    file << "idDepot,idVehicleType,idRoute,idVisit,idNode,type,distDriven,arrivalTime,departureTime,twWait,twSlackStart,twSlackEnd,latitude,longitude,qDel,qCol,iMix,iFill,loadMax\n";
    for (std::size_t i = 0; i < d_depotSols.size(); i++) {
        for (std::size_t j = 0; j < d_depotSols[i].d_vehTypeSols.size(); j++) {
            for (std::size_t k = 0; k < d_depotSols[i].d_vehTypeSols[j].d_routes.size(); k++) {
                float distDriven = 0;
                std::size_t idCurNode = i, idPrevNode = i;
                for (std::size_t l = 0; l < d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_visits.size(); l++) {
                    idCurNode = d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_visits[l].d_idNode;
                    distDriven += d_pGraph->getDist(j,idPrevNode,idCurNode);
                    file << d_depotSols[i].d_idDepot << ","
                         << d_depotSols[i].d_vehTypeSols[j].d_idVehicleType << ","
                         << d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_idRoute << ","
                         << d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_visits[l].d_idVisit << ","
                         << d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_visits[l].d_idNode << ","
                         << d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_visits[l].d_type << ","
                         << distDriven << ","
                         << d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_visits[l].d_timeArrival << ","
                         << d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_visits[l].d_timeDeparture << ","
                         << d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_visits[l].d_twWait << ","
                         << d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_visits[l].d_twSlackStart << ","
                         << d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_visits[l].d_twSlackEnd << ","
                         << d_pInst->d_nodes[d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_visits[l].d_idNode].d_latitude << ","
                         << d_pInst->d_nodes[d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_visits[l].d_idNode].d_longitude << ","
                         << d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_visits[l].d_qDel << ","
                         << d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_visits[l].d_qCol << ","
                         << d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_iMix << ","
                         << d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_iFill << ","
                         << d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_loadMax << "\n";
                    idPrevNode = idCurNode;
                }
            }
        }
    }
    file.close();
}

void Solution::writeJSON(std::string path) const {
    /* 
    Writes the solution into a JSON file
    */
   // Write the solution to a file
    std::ofstream file(path);
    if (file.is_open()) {
        file << "{\n";
        file << "  \"Instance\": \"" << d_pInst->d_name << "\",\n";
        file << "  \"Algorithm\": \"Heuristic\",\n";
        file << "  \"Integer Master Problem Solution\": {\n";
        file << "    \"Objective\": " << d_cost << ",\n";
        file << "    \"Variables\": [\n";
        int ctrRoutes = 0;
        for (std::size_t i = 0; i <  d_depotSols.size(); i++) {
            for (std::size_t j = 0; j <  d_depotSols[i].d_nVehicleTypes; j++) {
                for (std::size_t k = 0; k <  d_depotSols[i].d_vehTypeSols[j].d_routes.size(); k++) {
                    if (ctrRoutes > 0) {
                        file << ",\n";
                    }
                    ctrRoutes++;
                    const Route& route = d_depotSols[i].d_vehTypeSols[j].d_routes[k];
                    file << "      {\n";
                    file << "        \"weight\":1,\n";
                    file << "        \"nodes\": [";
                    for (std::size_t idVisit = 0; idVisit < route.d_nVisits; idVisit++) {
                        file << route.d_visits[idVisit].d_idNode;
                        if (idVisit < route.d_nVisits - 1) {
                            file << ", ";
                        }
                    }
                    file << "],\n";
                    file << "        \"scheduled lines\": [";
                    for (std::size_t idVisit = 1; idVisit < route.d_nVisits - 1; idVisit++) {
                        std::size_t idNode = route.d_visits[idVisit].d_idNode;
                        if (d_visitLocs.at(idNode).d_idSL == std::numeric_limits<std::size_t>::max()) {
                            file << "-1";
                        }
                        else {
                            file << d_visitLocs.at(idNode).d_idSL;
                        }
                        if (idVisit < route.d_nVisits - 2) {
                            file << ", ";
                        }
                    }
                    file << "]\n";
                    file << "      }";
                }
            }
        }
        file << "\n    ]\n  }\n}";
    }
    else {
        std::cerr << "Unable to open file for writing." << std::endl;
    }
}

std::size_t Solution::getNAreaInfeasible() const {
    /*
    Returns the number of infeasible requests in the solution
    */
    std::size_t nInfeasible = 0;
    // Iterate over all visits and check if the depot is equal to the assigned depot
    for (std::size_t i = 0; i < d_depotSols.size(); i++) {
        for (std::size_t j = 0; j < d_depotSols[i].d_nVehicleTypes; j++) {
            for (std::size_t k = 0; k < d_depotSols[i].d_vehTypeSols[j].d_nUsedVehicles; k++) {
                for (std::size_t l = 1; l < d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_nVisits-1; l++) {
                    if (d_pInst->d_nodes[d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_visits[l].d_idNode].d_idDepotAlloc != i) {
                        nInfeasible++;
                    }
                }
            }
        }
    }
    return nInfeasible;
}

DepotSolution::DepotSolution() {
    /*
    Empty constructor for the depot solution struct.
    */
    d_idDepot = std::numeric_limits<std::size_t>::max();
    d_nVehicleTypes = 0;
}

DepotSolution::DepotSolution(std::size_t idDepot, std::vector<VehicleTypeInfo> vehTypesInfo) :
                            d_idDepot(idDepot) {
    /*
    Constructor for the depot solution struct.
    */
    // Create empty routes for each vehicle type based on the available number of vehicles
    d_nVehicleTypes = 0;
    VehicleTypeInfo vehTypeInfo;
    for (std::size_t i = 0; i < vehTypesInfo.size(); i++) {
        vehTypeInfo = vehTypesInfo[i];
        if (vehTypeInfo.d_idDepot == idDepot) {
            d_vehTypeSols.push_back(VehicleTypeSolution(vehTypeInfo.d_idVehicleType,
                                                            idDepot,
                                                            vehTypeInfo.d_nVehicles));
            d_nVehicleTypes++;
        }
    }
}

VehicleTypeSolution::VehicleTypeSolution() {
    /*
    Empty constructor for the vehicle type solution struct.
    */
    d_idVehicleType = std::numeric_limits<std::size_t>::max();
    d_idDepot = std::numeric_limits<std::size_t>::max();
    d_nVehicles = std::numeric_limits<std::size_t>::max();
    d_nUsedVehicles = 0;
    d_penalty = 0;
}

VehicleTypeSolution::VehicleTypeSolution(std::size_t idVehicleType, std::size_t idDepot, std::size_t nVehicles) :
                                        d_idVehicleType(idVehicleType), d_idDepot(idDepot), d_nVehicles(nVehicles) {
    /*
    Constructor for the vehicle type solution struct.
    */
    d_nUsedVehicles = 0;
    d_routes.reserve(nVehicles);
    d_penalty = 0;
}

SLSolution::SLSolution() {
    /*
    Empty constructor for the SL solution struct.
    */
    d_idSL = std::numeric_limits<std::size_t>::max();
    d_qAssigned = 0;
    d_nAssigned = 0;
}

SLSolution::SLSolution(std::size_t idSL) : d_idSL(idSL) {
    /*
    Constructor for the SL solution struct.
    */
    d_qAssigned = 0;
    d_nAssigned = 0;
    d_assignments.reserve(30);
};


void SLSolution::assign(const Node &req) {
    /*
    Assigns a request to the SL.
    */
    // Add the request to the SL
    d_assignments.push_back(req.d_idNode);
    // Update the requested quantity
    d_qAssigned += req.d_qDel + req.d_qCol;
    // Update the number of requests
    d_nAssigned++;
    return;
}

void SLSolution::remove(const Node &req, std::unordered_map<std::size_t, VisitLoc> &visitLocs) {
    /*
    Removes a request from the SL.
    */
    // Remove the request from the SL
    std::size_t pos = visitLocs.at(req.d_idNode).d_posSL;
    d_assignments.erase(d_assignments.begin()+static_cast<int>(pos));
    // Update the requested quantity
    d_qAssigned -= (req.d_qDel + req.d_qCol);
    // Update the number of requests
    d_nAssigned--;
    // Update the positions of the requests in visitLocs
    for (std::size_t i = pos; i < d_nAssigned; i++) {
        visitLocs.at(d_assignments[i]).d_posSL--;
    }
    return;
}

float VehicleTypeSolution::checkOpen(const Node &req, int twDepotStart, int twDepotEnd, float excessiveRoutePenalty,
                                            const Graph &graph, const VehicleType &vehType) const {
    /*
    Takes a new customer node and determines if opening a route will be feasible
    */
    float costOpen = std::numeric_limits<float>::max();
    // Check if the order fits in the vehicle
    if (req.d_qCol + req.d_qDel > vehType.d_capacity) {
        return costOpen;
    }
    // Determine the distance to the request
    float distToReq = graph.getDist(vehType.d_idVehicleType,d_idDepot,req.d_idNode);
    float distToDepot = graph.getDist(vehType.d_idVehicleType,req.d_idNode,d_idDepot);
    // Determine the arrival time at the request
    int timeDrive = graph.getTime(vehType.d_idVehicleType,d_idDepot,req.d_idNode);
    int timeDriveBack = graph.getTime(vehType.d_idVehicleType,req.d_idNode,d_idDepot);
    // Check if the route time is feasible
    if (timeDrive + req.d_timeServ + timeDriveBack > vehType.d_timeMax) {
        return costOpen;
    }
    if (timeDrive + timeDriveBack > vehType.d_timeDriveMax) {
        return costOpen;
    }
    // Check if you can make the customer time window
    if (twDepotStart+timeDrive > req.d_twCustEnd) {
        return costOpen;
    }
    // Check if you can make the return time window
    int timeArrival = std::max(twDepotStart+timeDrive,req.d_twCustStart);
    if (timeArrival + req.d_timeServ + timeDriveBack > twDepotEnd) {
        return costOpen;
    }
    // Otherwise insert is feasible and calculate the cost
    costOpen = (distToReq + distToDepot) * vehType.d_costVariable + vehType.d_costFixed;
    if (d_nUsedVehicles >= d_nVehicles) {
        costOpen += excessiveRoutePenalty;
    }
    return costOpen;
}

void VehicleTypeSolution::openRoute(const Node &req, int twDepotStart, int twDepotEnd, float excessiveRoutePenalty,
                                    const Node &depot, const Graph &graph, const VehicleType &vehType) {
    /*
    Opens a new route for the given node and depot
    */
    // Reserve memory for the route and resize to three
    d_routes.push_back(Route(d_nUsedVehicles, d_idVehicleType, d_idDepot));
    d_routes[d_nUsedVehicles].d_visits.reserve(30);
    d_routes[d_nUsedVehicles].d_visits.resize(3);

    // Determine the drive time to the request
    int timeDrive = graph.getTime(vehType.d_idVehicleType,d_idDepot,req.d_idNode);
    int timeDriveBack = graph.getTime(vehType.d_idVehicleType,req.d_idNode,d_idDepot);

    // Determine the arrival time at the request
    int timeArrival = std::max(twDepotStart+timeDrive,req.d_twCustStart);
    int twWait = std::max(0, req.d_twCustStart - (twDepotStart+timeDrive));

    // Add the depot and request to the route
    d_routes[d_nUsedVehicles].d_visits[0] = Visit(depot.d_twDepotStart,twDepotStart,0,0,0,0,d_idDepot,req.d_qDel,0,-1);
    d_routes[d_nUsedVehicles].d_visits[1] = Visit(timeArrival,timeArrival+req.d_timeServ,twWait,timeArrival-req.d_twCustStart,req.d_twCustEnd-timeArrival-req.d_timeServ,1,req.d_idNode,0,req.d_qCol,req.d_type);
    d_routes[d_nUsedVehicles].d_visits[2] = Visit(twDepotEnd,depot.d_twDepotEnd,twDepotEnd-(timeArrival+req.d_timeServ+timeDriveBack),0,0,2,d_idDepot,0,req.d_qCol,-1);

    // Update route parameters
    d_routes[d_nUsedVehicles].d_nVisits = 3;
    d_routes[d_nUsedVehicles].d_dist = graph.getDist(vehType.d_idVehicleType,d_idDepot,req.d_idNode) + graph.getDist(vehType.d_idVehicleType,req.d_idNode,d_idDepot);
    d_routes[d_nUsedVehicles].d_cost = d_routes[d_nUsedVehicles].d_dist*vehType.d_costVariable + vehType.d_costFixed;
    d_routes[d_nUsedVehicles].d_timeDrive = timeDrive + timeDriveBack;
    d_routes[d_nUsedVehicles].d_timeTravel = d_routes[d_nUsedVehicles].d_timeDrive + req.d_timeServ;
    d_routes[d_nUsedVehicles].d_loadMax = req.d_qDel+req.d_qCol;
    d_routes[d_nUsedVehicles].d_iMix = 1;
    d_routes[d_nUsedVehicles].d_timeDeparture = timeArrival - timeDrive; // Denote the latest possible departure time
    d_routes[d_nUsedVehicles].d_timeArrival = timeArrival+req.d_timeServ+timeDriveBack; // Denote the earliest possible arrival time
    if (req.d_qCol > vehType.d_beta) {
        d_routes[d_nUsedVehicles].d_iFill = 1;
    }
    else {
        d_routes[d_nUsedVehicles].d_iFill = 2;
    }
    if (req.d_qDel > vehType.d_alpha) {
        d_routes[d_nUsedVehicles].d_iMix = 1;
    }
    else {
        d_routes[d_nUsedVehicles].d_iMix = 0;
    }
    d_nUsedVehicles++;
    if (d_nUsedVehicles > d_nVehicles) {
        d_penalty = static_cast<float>(d_nUsedVehicles-d_nVehicles) * excessiveRoutePenalty;
    }
    return;
}

void VehicleTypeSolution::closeRoute(std::size_t idRoute, float excessiveRoutePenalty,  std::unordered_map<std::size_t, VisitLoc> &visitLocs) {
    /*
    Closes a route and removes it from the solution when all nodes are removed
    */
    // Remove the route from the solution
    d_routes.erase(d_routes.begin()+static_cast<int>(idRoute));
    d_nUsedVehicles--;

    if (d_penalty > 0) {
        d_penalty -= excessiveRoutePenalty;
    }
    // Update all route indices as well as the node location hashmap
    for (std::size_t i=idRoute;i<d_nUsedVehicles;i++) {
        d_routes[i].d_idRoute = i;
        if (d_routes[i].d_nVisits == 0) {
            continue;
        }
        for (std::size_t j=1;j<d_routes[i].d_nVisits-1;j++) {
            visitLocs[d_routes[i].d_visits[j].d_idNode].d_idRoute = i;
            visitLocs[d_routes[i].d_visits[j].d_idNode].d_idVisit = j;
        }
    }
}

std::size_t Solution::hash() const {
    /*
    Creates a hash for the solution based on the hash of each route
    */
    std::size_t hashValue = 0;
    std::hash<unsigned short> hasher;
    for (const DepotSolution& depotSol : d_depotSols) {
        for (const VehicleTypeSolution& vehTypeSol : depotSol.d_vehTypeSols) {
            for (const Route& route : vehTypeSol.d_routes) {
                for (const Visit& visit : route.d_visits) {
                    hashValue ^= hasher(visit.d_idNode) + 0x9e3779b9 + (hashValue << 6) + (hashValue >> 2);
                }
            }
        }
    }
    return hashValue;

}