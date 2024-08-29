#include "alnsSolution.hpp"

bool ALNSSolution::isValid(bool destroyed, Duals duals) const {
    /*
    performs sanity checks on the solution, returns true if the solution is valid
    Does not ensure feasibility, only checks if the solution is valid
    */
    bool valid = true;
    double costRoutes = 0; // Cost based on the sum of all route costs
    double costDist = 0; // Cost based on the sum of all route distances
    double costSL = 0; // Cost based on the sum of all scheduled line costs
    unsigned short nVisits = 0; // Number of visits
    int timeDriveTot = 0; // Total travel time
    std::vector<unsigned short> visitedCustNodes; // Set to fill with visited nodes
    double reducedCostCustomers = 0;

    visitedCustNodes.reserve(d_pInst->d_nNodes);
    // Check if depot solution lengths are correct
    if (d_depotSols.size() != d_pInst->d_nDepots) {
        std::cout << "Solution has incorrect number of depots\n";
        return valid = false;
    }
    // Check if the number of unassigned requests is 0
    if (!destroyed && d_reqUnassigned.size() != 0) {
        std::cout << "Unassigned requests has incorrect number of requests\n";
        return valid = false;
    }
    // Iterate over scheduled lines
    for (unsigned short i = 0; i < d_pInst->d_nSL; i++) {
        // Check if the depot solution makes sense
        if (d_SLSols[i].d_idSL != i) {
            std::cout << "Scheduled line solution has incorrect scheduled line id\n";
            return valid = false;
        }
        // Check if the number of assigned requests matches the number of assigned requests
        if (d_SLSols[i].d_nAssigned != d_SLSols[i].d_assignments.size()) {
            std::cout << "Scheduled line solution has incorrect number of assigned requests\n";
            return valid = false;
        }
        // Iterate over assigned requests
        unsigned short qSL = 0; // Quantity on a scheduled line
        for (unsigned short j = 0; j < d_SLSols[i].d_nAssigned; j++) {
            qSL += d_pInst->d_nodes[d_SLSols[i].d_assignments[j]].d_qCol + d_pInst->d_nodes[d_SLSols[i].d_assignments[j]].d_qDel;
            // Check if the departure time of the scheduled line is before the international arrival time of the request
            if (d_pInst->d_scheduledLines[i].d_timeDep < d_pInst->d_nodes[d_SLSols[i].d_assignments[j]].d_twDepotStart) {
                std::cout << "Scheduled line departs before international arrival time\n";
                return valid = false;
            }
        }
        // Check if the quantity on the scheduled line matches the quantity on the scheduled line solution
        if (qSL != d_SLSols[i].d_qAssigned) {
            std::cout << "Scheduled line solution has incorrect quantity on scheduled line\n";
            return valid = false;
        }
        // Update total cost of scheduled lines
        costSL += static_cast<double>(d_SLSols[i].d_qAssigned)*(d_pInst->d_scheduledLines[i].d_costRequest - duals.d_sl[i]);
    }
    // Iterate over depot solutions
    for (unsigned short i = 0; i < d_pInst->d_nDepots; i++) {
        // Check if depot solution has correct depot id
        if (d_depotSols[i].d_idDepot != i) {
            std::cout << "Depot solution has incorrect depot id\n";  
            return valid = false;
        }
        // Iterate over vehicle type solutions
        for (unsigned short j = 0; j < d_depotSols[i].d_nVehicleTypes; j++) {
            const VehicleTypeSolution& curVehTypeSol = d_depotSols[i].d_vehTypeSols[j];
            // Check if vehicle type solution has correct depot id
            if (curVehTypeSol.d_idDepot != i) {
                std::cout << "Vehicle type solution has incorrect depot id\n";
                return valid = false;
            }
            // Reset counters
            double distVeh = 0;
            unsigned short nUsedVeh = 0;
            // Iterate over routes
            for (unsigned short k = 0; k < curVehTypeSol.d_nUsedVehicles; k++) {
                // Create pointer to the current route
                const Route& curRoute = curVehTypeSol.d_routes[k];
                // Reset the distance of the route
                double routeDist = 0;
                // Check if route has correct depot id
                if (curRoute.d_idDepot != i) {
                    std::cout << "Depot " << i << " vehicle " << j << " route " << k << " has incorrect depot id\n";
                    return valid = false;
                }
                // Check if route has correct vehicle type id
                if (curRoute.d_idVehicleType != j) {
                    std::cout << "Depot " << i << " vehicle " << j << " route " << k << " has incorrect vehicle type id\n";
                    return valid = false;
                }
                // Check if the route is empty
                if (curRoute.d_nVisits == 0) {
                    std::cout << "Number of used routes is incorrect\n";
                }
                if (curRoute.d_visits.size() != 0) {
                    nUsedVeh++;
                }
                // Check if the route departure time is positive
                if (curRoute.d_timeDeparture < 0) {
                    std::cout << "Depot " << i << " vehicle " << j << " route " << k << " departure time is negative\n";
                    return valid = false;
                }
                // Check if arrival time is after departure time
                if (curRoute.d_timeArrival < curRoute.d_timeDeparture) {
                    std::cout << "Depot " << i << " vehicle " << j << " route " << k << " arrival time is before departure time\n";
                    return valid = false;
                }
                // Check if the departure time is consisten with the first visit
                if (curRoute.d_timeDeparture != curRoute.d_visits[1].d_timeArrival - d_pGraph->getTime(j,i,curRoute.d_visits[1].d_idNode)) {
                    std::cout << "Depot " << i << " vehicle " << j << " route " << k << " departure time is inconsistent with first visit\n";
                    return valid = false;
                }
                // Check if the arrival time is consistent with the last visit
                if (curRoute.d_timeArrival != curRoute.d_visits[curRoute.d_nVisits-2].d_timeDeparture + d_pGraph->getTime(j,curRoute.d_visits[curRoute.d_nVisits-2].d_idNode,i)) {
                    std::cout << "Depot " << i << " vehicle " << j << " route " << k << " arrival time is inconsistent with last visit\n";
                    return valid = false;
                }
                nVisits = 0;
                timeDriveTot = 0;
                // Iterate over visits
                for (unsigned short l = 1; l < curRoute.d_nVisits; l++) {
                    // Create pointer to the current visit
                    const Visit& curVisit = curRoute.d_visits[l];
                    const Visit& prevVisit = curRoute.d_visits[l-1];
                    // If the visit is a depot, skip
                    if (curVisit.d_idNode != i) {
                        // Check if the request has a visitLoc location
                        if (d_visitLocs.find(curVisit.d_idNode) == d_visitLocs.end()) {
                            std::cout << "Node " << curVisit.d_idNode <<  " has no visitLoc location\n";
                            return valid = false;
                        }
                        // Check if the visitLoc location of this request is correct
                        if (d_visitLocs.at(curVisit.d_idNode).d_idDepot != i ||
                            d_visitLocs.at(curVisit.d_idNode).d_idVehicleType != j ||
                            d_visitLocs.at(curVisit.d_idNode).d_idRoute != k ||
                            d_visitLocs.at(curVisit.d_idNode).d_idVisit != l) {
                            std::cout << "visitLoc location of request " << curVisit.d_idNode << " is incorrect\n";
                            return valid = false;
                        }
                    }
                    // Check if the request is also assigned to a scheduled line if necessary
                    if (d_pInst->d_nodes[curVisit.d_idNode].d_idDepot != i && d_pInst->d_nodes[curVisit.d_idNode].d_idDepot < d_pInst->d_nDepots) {
                        if (d_visitLocs.at(curVisit.d_idNode).d_idSL == std::numeric_limits<unsigned short>::max()) {
                            std::cout << "Visit " << curVisit.d_idNode << " is not assigned to a scheduled line\n";
                            return valid = false;
                        }
                        // Check if the departure time of the route is after the arrival of the SL
                        if (curVisit.d_type == 1 && curRoute.d_timeDeparture < d_pInst->d_scheduledLines[d_visitLocs.at(curVisit.d_idNode).d_idSL].d_timeArr) {
                            std::cout << "Depot " << i << " vehicle " << j << " route " << k << " departs before scheduled line arrives\n";
                            return valid = false;
                        }
                        // Check if the arrival time of the route is before the departure of the SL
                        if (curVisit.d_type == 0 && curRoute.d_timeArrival > d_pInst->d_scheduledLines[d_visitLocs.at(curVisit.d_idNode).d_idSL].d_timeDep) {
                            std::cout << "Depot " << i << " vehicle " << j << " route " << k << " arrives after scheduled line departs\n";
                            return valid = false;
                        }
                    }
                    // Check if the time windows for the international scheduled lines are met
                    if (d_pInst->d_nodes[curVisit.d_idNode].d_twDepotStart > curRoute.d_timeDeparture) {
                        std::cout << "Depot " << i << " vehicle " << j << " route " << k << " departs before international scheduled line arrives\n";
                        return valid = false;
                    }
                    if (d_pInst->d_nodes[curVisit.d_idNode].d_twDepotEnd < curRoute.d_timeArrival && d_pInst->d_nodes[curVisit.d_idNode].d_idDepot < d_pInst->d_nDepots) {
                        std::cout << "Depot " << i << " vehicle " << j << " route " << k << " arrives after international scheduled line departs\n";
                        return valid = false;
                    }
                    // Check if visit has correct visit id
                    if (curVisit.d_idVisit != l) {
                        std::cout << "Visit has incorrect visit id\n";
                        return valid = false;
                    }
                    // Check if the node id is valid
                    if (curVisit.d_idNode >= d_pInst->d_nNodes) {
                        std::cout << "Visit has invalid node id\n";
                        return valid = false;
                    }
                    // Check if waiting time is non-negative
                    if (curVisit.d_twWait < 0) {
                        std::cout << "Visit has negative waiting time\n";
                        return valid = false;
                    }
                    // Check if arrival time is non-negative
                    if (curVisit.d_timeArrival < 0) {
                        std::cout << "Visit has negative arrival time\n";
                        return valid = false;
                    }
                    // Check if departure time is after arrival time
                    if (curVisit.d_timeDeparture < curVisit.d_timeArrival) {
                        std::cout << "Visit has departure time before arrival time\n";
                        return valid = false;
                    }
                    // Check driving time constraints
                    int timeDrive = d_pGraph->getTime(j,prevVisit.d_idNode,curVisit.d_idNode);
                    if (curVisit.d_timeArrival - prevVisit.d_timeDeparture - timeDrive - curVisit.d_twWait != 0) {
                        std::cout << "Depot " << i << " vehicle " << j << " route " << k << " visit " << l << " has inconsistent driving time\n";
                        return valid = false;
                    }
                    // Add the distance to the route distance
                    routeDist += d_pGraph->getDist(curVehTypeSol.d_idVehicleType,prevVisit.d_idNode,curVisit.d_idNode);  
                    nVisits++;
                    timeDriveTot += timeDrive;
                    // Check if the visit is a depot node 
                    if (curVisit.d_idNode != i) {
                        visitedCustNodes.push_back(curVisit.d_idNode);
                    }
                    // Update the reduced cost of the customers
                    if (curVisit.d_idNode >= d_pInst->d_nDepots) {
                        reducedCostCustomers += duals.d_customer[curVisit.d_idNode - duals.d_nDepots]; 
                    }    
                }
                // Check if the number of visits is correct
                if (nVisits != curRoute.d_nVisits-1) {
                    std::cout << "Route has incorrect number of visits\n";
                    return valid = false;
                }
                // Check if the drive time is correct
                if (timeDriveTot != curRoute.d_timeDrive) {
                    std::cout << "Route has incorrect driving time\n";
                    return valid = false;
                }
                // Check if the distance is correct
                if (std::abs(routeDist - curRoute.d_dist) > d_pSettings->d_tolerance) {
                    std::cout << "Depot " << i << " vehicle " << j << " route " << k << " has incorrect distance, off by " << routeDist - curRoute.d_dist << " \n";
                    return valid = false;
                }
                costRoutes += curRoute.d_cost;
                distVeh += curRoute.d_dist;
            }
            // Check if the number of used vehicles is correct
            if (nUsedVeh != d_depotSols[i].d_vehTypeSols[j].d_nUsedVehicles) {
                std::cout << "Vehicle type solution has incorrect number of used vehicles\n";
                return valid = false;
            }
            // Update distance-based cost for the vehicle type solution
            costDist += distVeh*d_pInst->d_vehTypes[j].d_costVariable+
                        static_cast<double>(curVehTypeSol.d_nUsedVehicles)*(d_pInst->d_vehTypes[j].d_costFixed - duals.d_vehicle[i*duals.d_nDepots + j]);
        }
    }
    // Verify that the distance-based calculation is equal to the cost-based calculation
    if (std::abs(costDist + costSL - reducedCostCustomers - d_cost) > d_pSettings->d_tolerance) {
        std::cout << "Distance-based calculation is off cost-based calculation by " <<  std::abs(costDist + costSL - reducedCostCustomers - d_cost) <<"\n";
        return valid = false;
    }
    // Verify that the cost of the depot solutions is equal to the cost of the solution
    if (std::abs(costRoutes - d_cost) > d_pSettings->d_tolerance) {
        std::cout << "Cost of depot solutions is not equal to cost of solution\n";
        return valid = false;
    }
    std::set<unsigned short> visitedCustNodesSet(visitedCustNodes.begin(), visitedCustNodes.end());
    // Verify that the number of unique nodes is equal to the number of nodes
    if (!destroyed && visitedCustNodesSet.size() != d_pInst->d_nCustNodes) {
        std::cout << "Number of customer nodes is not equal to the number of customers\n";
        return valid = false;
    }
    // Verify that there a no duplicate nodes in the solution
    if (visitedCustNodes.size() != visitedCustNodesSet.size()) {
        std::cout << "Duplicate nodes in the solution\n";
        return valid = false;
    }
    return valid;
}

bool ALNSSolution::isFeasible() const {
    /*
    Performs feasibility checks on the solution.
    */
    bool feasible = true;
    // Iterate over scheduled lines
    for (unsigned short i = 0; i < d_pInst->d_nSL; i++) {
        // Check if the quantity on the scheduled line is not exceeded
        if (d_SLSols[i].d_qAssigned > d_pInst->d_scheduledLines[i].d_capacity) {
            std::cout << "Scheduled line capacity constraint violated\n";
            return feasible = false;
        }
    }
    // Iterate over depot solutions
    for (unsigned short i = 0; i < d_pInst->d_nDepots; i++) {
        // Iterate over vehicle type solutions
        for (unsigned short j = 0; j < d_pInst->d_nVehTypes; j++) {
            // Iterate over routes
            for (unsigned short k = 0 ; k < d_depotSols[i].d_vehTypeSols[j].d_nUsedVehicles; k++) {
                // Create pointer to the current route
                const Route& curRoute = d_depotSols[i].d_vehTypeSols[j].d_routes[k];
                // Check driving and travel time
                if (curRoute.d_timeDrive > d_pInst->d_vehTypes[j].d_timeDriveMax) {
                    std::cout << "Depot " << i << " vehicle " << j << " route " << k << " exceeds maximum driving time\n";
                    return feasible = false;
                }
                if (curRoute.d_timeTravel > d_pInst->d_vehTypes[j].d_timeMax) {
                    std::cout << "Depot " << i << " vehicle " << j << " route " << k << " exceeds maximum travel time\n";
                    return feasible = false;
                }
                // Iterate over visits
                for (unsigned short l = 1; l < curRoute.d_nVisits-1; l++) {
                    // Create pointers to the current and previous visit
                    const Visit& curVisit = curRoute.d_visits[l];
                    // Check capacity constraints
                    if (curVisit.d_qDel + curVisit.d_qCol > d_pInst->d_vehTypes[j].d_capacity) {
                        std::cout << "Capacity constraint violated.\n";
                        return feasible = false;
                    }
                    if (curVisit.d_type == 0 && curVisit.d_qDel > d_pInst->d_vehTypes[j].d_alpha) {
                        std::cout << "Mixing level constraint violated\n";
                        return feasible = false;
                    }
                    if (curVisit.d_type == 1 && curVisit.d_qCol > d_pInst->d_vehTypes[j].d_beta) {
                        std::cout << "Fill up to level constraint violated\n";
                        return feasible = false;
                    }
                    // Check time constraints
                    if (curVisit.d_timeArrival < d_pInst->d_nodes[curVisit.d_idNode].d_twCustStart) {
                        std::cout << "Depot " << i << " vehicle " << j << " route " << k << " visit " << l << " arrives before time window\n";
                        return feasible = false;
                    }
                    if (curVisit.d_timeArrival > d_pInst->d_nodes[curVisit.d_idNode].d_twCustEnd) {
                        std::cout << "Depot " << i << " vehicle " << j << " route " << k << " visit " << l << " arrives after time window\n";
                        return feasible = false;
                    }        
                }
            }
        }
    }
    return feasible;
}