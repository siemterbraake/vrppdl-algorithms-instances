#include "metaheuristic.hpp"

void Metaheuristic::destroyDynamic(unsigned short idAnchor, double routeCoef) {
    /*
    Destroys a solution by removing nodes based on the dynamic destroy operation. 
    Destroy operator #0
    */
    const VisitLoc locAnchor = d_tempSol.d_visitLocs.at(idAnchor);
    Route &route = d_tempSol.d_depotSols[locAnchor.d_idDepot].d_vehTypeSols[locAnchor.d_idVehicleType].d_routes[locAnchor.d_idRoute];
    unsigned short nRemRoute = std::min(static_cast<unsigned short>(std::round(static_cast<double>(d_sizeNeighborhood)*routeCoef)), static_cast<unsigned short>(route.d_nVisits-2));
    unsigned short ctrRemoved = 0;
    if (nRemRoute == route.d_nVisits-2) {
        // Remove the entire route
        d_tempSol.removeRoute(locAnchor.d_idDepot, locAnchor.d_idVehicleType, locAnchor.d_idRoute, ctrRemoved);
    }
    else {
        // Perform string removal centered around the anchor node
        // Store the visit id of the anchor node
        unsigned short idVisit = locAnchor.d_idVisit;
        while (ctrRemoved < nRemRoute) {
            if (route.d_visits.size() < 3) {
                break;
            }           
            // Remove adjecent nodes from the route
            d_tempSol.removeNode(route.d_visits[idVisit].d_idNode, d_duals);
            ctrRemoved++;
            // Check if crtRemoved is divisible by 2, and the limits are not exceeded
            if (ctrRemoved % 2 == 0 && idVisit > 1) {
                idVisit--;
            }
            if (idVisit >= route.d_nVisits-1) {
                idVisit--;
            }
        }
    }
    // Perform nn-remove on the remaining nodes
    auto nearestNodes = d_graph.getNearestNodes(idAnchor);
    while (ctrRemoved < d_sizeNeighborhood) {
        // Check if the node is in visitLocs
        unsigned short idNode = *nearestNodes.first;
        if (d_tempSol.d_visitLocs.find(idNode) != d_tempSol.d_visitLocs.end()) {
            d_tempSol.removeNode(idNode, d_duals);
            ctrRemoved++;
        }
        ++nearestNodes.first;
    }
}

void Metaheuristic::destroyRandomRoute() {
    /*
    Destroys a solution by removing random routes until the neighborhood size is reached
    Destroy operator #1
    */
    unsigned short ctrRemoved = 0;
    while (ctrRemoved < d_sizeNeighborhood) {
        // Select a random route, making sure it is not empty
        bool found = false;
        unsigned short max = std::numeric_limits<unsigned short>::max();
        unsigned short idDepot = max, idVehicleType = max, idRoute = max;
        while (!found) {
            idDepot = std::uniform_int_distribution<unsigned short>(0,d_inst.d_nDepots-1)(d_gen);
            idVehicleType = std::uniform_int_distribution<unsigned short>(0,d_tempSol.d_depotSols[idDepot].d_nVehicleTypes-1)(d_gen);
            if (d_tempSol.d_depotSols[idDepot].d_vehTypeSols[idVehicleType].d_nUsedVehicles > 0) {
                idRoute = std::uniform_int_distribution<unsigned short>(0,d_tempSol.d_depotSols[idDepot].d_vehTypeSols[idVehicleType].d_nUsedVehicles-1)(d_gen);
                if (d_tempSol.d_depotSols[idDepot].d_vehTypeSols[idVehicleType].d_routes[idRoute].d_nVisits > 2) {
                    found = true;
                }
            }
        }
        // Remove the route
        d_tempSol.removeRoute(idDepot, idVehicleType, idRoute, ctrRemoved);
    }
}

void Metaheuristic::destroyGreedyRoute(bool noise) {
    /*
    Destoys the route with the highest cost per node until the neighborhood size is reached.
    Destroy operator #2 (with noise)
    */
    // Determine the number of routes
    unsigned short nRoutes = 0;
    for (unsigned short i = 0; i < d_inst.d_nDepots; i++) {
        for (unsigned short j = 0; j < d_tempSol.d_depotSols[i].d_nVehicleTypes; j++) {
            nRoutes += d_tempSol.d_depotSols[i].d_vehTypeSols[j].d_nUsedVehicles;
        }
    }
    // Create a vector of all routes
    std::vector<std::tuple<unsigned short,unsigned short,unsigned short,double>> routeCosts(nRoutes);
    // Loop over the routes and store the cost per node
    unsigned short ctr = 0;
    for (unsigned short i = 0; i < d_inst.d_nDepots; i++) {
        for (unsigned short j = 0; j < d_tempSol.d_depotSols[i].d_nVehicleTypes; j++) {
            for (unsigned short k = 0; k < d_tempSol.d_depotSols[i].d_vehTypeSols[j].d_nUsedVehicles; k++) {
                double costRoute = d_tempSol.d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_cost;
                if (noise) {
                    costRoute += std::uniform_real_distribution<double>(-1,1)(d_gen)*d_settings.d_noiseLevel*costRoute;
                }
                double costPerNode = costRoute/static_cast<double>(d_tempSol.d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_nVisits-2);
                routeCosts[ctr] = std::make_tuple(i,j,k,costPerNode);
                ctr++;
            }
        }
    }
    // Sort the vector based on the fourth element (cost per node)
    std::sort(routeCosts.begin(), routeCosts.end(), [](const std::tuple<unsigned short,unsigned short,unsigned short,double> &a, const std::tuple<unsigned short,unsigned short,unsigned short,double> &b) {
        return std::get<3>(a) > std::get<3>(b);
    });
    unsigned short ctrRemoved = 0;
    while (ctrRemoved < d_sizeNeighborhood) {
        // Select the route with the highest cost per node
        unsigned short idDepot = std::get<0>(routeCosts[0]);
        unsigned short idVehicleType = std::get<1>(routeCosts[0]);
        unsigned short idRoute = std::get<2>(routeCosts[0]);
        // Remove the route
        d_tempSol.removeRoute(idDepot, idVehicleType, idRoute, ctrRemoved);
        // Remove the route from the vector
        routeCosts.erase(routeCosts.begin());
        // Update the route indices of all routes at the given depot and vehicle type
        for (unsigned short i = 0; i < routeCosts.size(); i++) {
            if (std::get<0>(routeCosts[i]) == idDepot && 
                std::get<1>(routeCosts[i]) == idVehicleType && 
                std::get<2>(routeCosts[i]) > idRoute) {
                std::get<2>(routeCosts[i])--;
            }
        }
    }
}

void Metaheuristic::destroyWaiting() {
    /*
    Destroys nodes that cause waiting time in the current solution.
    Removes random nodes if there is no waiting time
    Destroy operator #4
    */
    // Create a tuple with node id and waiting time if waiting time is positive
    std::vector<std::tuple<unsigned short,int>> waitingNodes;
    waitingNodes.reserve(d_inst.d_nCustNodes);
    for (unsigned short i = 0; i < d_inst.d_nDepots; i++) {
        DepotSolution &depotSol = d_tempSol.d_depotSols[i];
        for (unsigned short j = 0; j < depotSol.d_nVehicleTypes; j++) {
            VehicleTypeSolution &vehTypeSol = depotSol.d_vehTypeSols[j];
            for (unsigned short k = 0; k < vehTypeSol.d_nUsedVehicles; k++) {
                Route &route = vehTypeSol.d_routes[k];
                // Start at second visit as the first always has waiting time
                for (unsigned short l = 2; l < route.d_nVisits-1; l++) {
                    unsigned short idNode = route.d_visits[l].d_idNode;
                    if (route.d_visits[l].d_twWait> 0) {
                        waitingNodes.push_back(std::make_tuple(idNode,route.d_visits[l].d_twWait));
                    }
                }
            }
        }
    }
    // Sort the nodes based on the waiting time
    std::sort(waitingNodes.begin(), waitingNodes.end(), [](const std::tuple<unsigned short,int> &a, const std::tuple<unsigned short,int> &b) {
        return std::get<1>(a) > std::get<1>(b);
    });
    // Remove the nodes with the highest waiting time
    unsigned short ctrRemoved = 0;
    for (unsigned short i = 0; i < waitingNodes.size(); i++) {
        d_tempSol.removeNode(std::get<0>(waitingNodes[i]), d_duals);
        ctrRemoved++;
        if (ctrRemoved == d_sizeNeighborhood) {
            break;
        }
    }
    // If the neighborhood size is not reached, remove random nodes
    if (ctrRemoved < d_sizeNeighborhood) {
        d_sizeNeighborhood -= ctrRemoved;
        destroyRandom();
    }
}

void Metaheuristic::destroyRandom(){
    /*
    Destroys a solution by removing n random requests
    Only used as fallback for destroyExcessive and destroyWaiting
    */
    // Create a vector of all customer requests
    std::vector<unsigned short> idCustNodes(d_inst.d_nCustNodes);
    std::iota(idCustNodes.begin(), idCustNodes.end(), d_inst.d_nDepots);
    // Remove customer ids that are in reqUnassigned
    for (unsigned short i = 0; i < d_tempSol.d_reqUnassigned.size(); i++) {
        idCustNodes.erase(std::remove(idCustNodes.begin(), idCustNodes.end(), d_tempSol.d_reqUnassigned[i]), idCustNodes.end());
    }
    // Shuffle the numbers
    std::shuffle(idCustNodes.begin(), idCustNodes.end(), d_gen);
    // Resize to neighborhood size
    idCustNodes.resize(d_sizeNeighborhood);

    // Remove the requests from the solution
    for (unsigned short i = 0; i < d_sizeNeighborhood; i++) {
        d_tempSol.removeNode(idCustNodes[i], d_duals);
    }
}

void Metaheuristic::destroyGreedy(bool noise) {
    /* 
    Destroys a solution by removing the nodes with the highest removal gain. 
    THIS OPERATOR IS NOT USED
    */
    // Loop over the routes and find the longest cust-to-cust edges
    std::vector<double> remGain(d_inst.d_nNodes);
    double remGainTemp = 0;
    for (unsigned short i = 0; i < d_inst.d_nCustNodes; i++) {
        // Skip the depots 
        if (i < d_inst.d_nDepots) {
            remGain[i] = -1;
            continue;
        }
        if (d_tempSol.d_visitLocs.find(i) != d_tempSol.d_visitLocs.end()) {
            const VisitLoc loc = d_tempSol.d_visitLocs.at(i);
            // Find previous node
            unsigned short idPrevNode = d_tempSol.d_depotSols[loc.d_idDepot].d_vehTypeSols[loc.d_idVehicleType].d_routes[loc.d_idRoute].d_visits[loc.d_idVisit-1].d_idNode;
            // Find next node
            unsigned short idNextNode = d_tempSol.d_depotSols[loc.d_idDepot].d_vehTypeSols[loc.d_idVehicleType].d_routes[loc.d_idRoute].d_visits[loc.d_idVisit+1].d_idNode;
            // Find the removal gain
            remGainTemp = d_graph.getDist(loc.d_idVehicleType,idPrevNode,i) +
                          d_graph.getDist(loc.d_idVehicleType,i,idNextNode) -
                          d_graph.getDist(loc.d_idVehicleType,idPrevNode,idNextNode);
            // Add SL cost if necessary
            if (loc.d_idSL != std::numeric_limits<unsigned short>::max()) {
                remGainTemp += d_inst.d_scheduledLines[loc.d_idSL].d_costRequest*static_cast<double>(d_inst.d_nodes[i].d_qCol+d_inst.d_nodes[i].d_qDel);
            }
            // Add noise if necessary
            if (noise) {
                remGainTemp += std::uniform_real_distribution<double>(-1,1)(d_gen)*d_settings.d_noiseLevel*remGainTemp;
            }
            // Add to the vector
            remGain[i] = remGainTemp;
            // COSTS ARE NOT THE ACTUAL COSTS HERE SO NEEDS TO BE RE-EVALUATED AT ACTUAL REMOVAL
        }
        else {
            std::cout << "WARNING: Request "+std::to_string(i)+" not found in solution. Skipping...\n";
        }
    }
    // make n loop over the vector and remove the highest gain nodes
    for (unsigned short i = 0; i < d_sizeNeighborhood; i++) {
        // Find the highest gain node
        unsigned short idNode = static_cast<unsigned short>(std::distance(remGain.begin(), std::max_element(remGain.begin(), remGain.end())));
        VisitLoc loc = d_tempSol.d_visitLocs.at(idNode);
        // Check if the route has only three visits
        bool removedRoute = d_tempSol.d_depotSols[loc.d_idDepot].d_vehTypeSols[loc.d_idVehicleType].d_routes[loc.d_idRoute].d_nVisits == 3;
        d_tempSol.removeNode(idNode, d_duals);
        // Set the removal gain to -1
        remGain[idNode] = -1;
        // Update the removal gains of the nodes that are affected
        if (removedRoute) {
            // Case where the route is removed, no other nodes are affected
            continue;
        }
        Route& routeAffected = d_tempSol.d_depotSols[loc.d_idDepot].d_vehTypeSols[loc.d_idVehicleType].d_routes[loc.d_idRoute];
        if (loc.d_idVisit == 1) {
            // Case where the first node is removed, only the second node is affected
            unsigned short idNodeAffected = routeAffected.d_visits[1].d_idNode;
            remGain[idNodeAffected] = d_graph.getDist(loc.d_idVehicleType,loc.d_idDepot,idNodeAffected) +
                                      d_graph.getDist(loc.d_idVehicleType,idNodeAffected,routeAffected.d_visits[2].d_idNode) - 
                                      d_graph.getDist(loc.d_idVehicleType,loc.d_idDepot,routeAffected.d_visits[2].d_idNode);
        }
        else if (loc.d_idVisit == routeAffected.d_nVisits-1) {
            // Case where the last node is removed, only the second to last node is affected
            unsigned short idNodeAffected = routeAffected.d_visits[routeAffected.d_nVisits-1].d_idNode;
            remGain[idNodeAffected] = d_graph.getDist(loc.d_idVehicleType,routeAffected.d_visits[routeAffected.d_nVisits-2].d_idNode,idNodeAffected) +
                                      d_graph.getDist(loc.d_idVehicleType,idNodeAffected,loc.d_idDepot) - 
                                      d_graph.getDist(loc.d_idVehicleType,routeAffected.d_visits[routeAffected.d_nVisits-2].d_idNode,loc.d_idDepot);
        }
        else {
            // Case where a node in the middle is removed, the previous and next node are affected
            unsigned short idNodeAffectedPrev = routeAffected.d_visits[loc.d_idVisit-1].d_idNode;
            unsigned short idNodeAffectedNext = routeAffected.d_visits[loc.d_idVisit].d_idNode;
            remGain[idNodeAffectedPrev] = d_graph.getDist(loc.d_idVehicleType,routeAffected.d_visits[loc.d_idVisit-2].d_idNode,idNodeAffectedPrev) +
                                          d_graph.getDist(loc.d_idVehicleType,idNodeAffectedPrev,idNodeAffectedNext) - 
                                          d_graph.getDist(loc.d_idVehicleType,routeAffected.d_visits[loc.d_idVisit-2].d_idNode,idNodeAffectedNext);
            remGain[idNodeAffectedNext] = d_graph.getDist(loc.d_idVehicleType,idNodeAffectedPrev,idNodeAffectedNext) +
                                          d_graph.getDist(loc.d_idVehicleType,idNodeAffectedNext,routeAffected.d_visits[loc.d_idVisit+1].d_idNode) - 
                                          d_graph.getDist(loc.d_idVehicleType,idNodeAffectedPrev,routeAffected.d_visits[loc.d_idVisit+1].d_idNode);
        }
    }
}
