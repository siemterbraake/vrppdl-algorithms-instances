#include "metaheuristic.hpp"

void Metaheuristic::repairGreedy(bool noise) {
    /* 
    Repairs a solution using greedy insert
    Repair operator #0 (without noise) and #1 (with noise)
    */
    // Shuffle the unassigned requests
    std::shuffle(d_tempSol.d_reqUnassigned.begin(), d_tempSol.d_reqUnassigned.end(), d_gen);
    while (d_tempSol.d_reqUnassigned.size()> 0) {
        // select the first unassigned request
        unsigned short iReq = d_tempSol.d_reqUnassigned[0];
        const Node &req = d_inst.d_nodes[iReq];

        // find the best insertion position
        unsigned short max = std::numeric_limits<unsigned short>::max();
        unsigned short bestDepot = max, bestSL = max, bestVehicle = max, bestRoute = max, bestPos = max;
        double costInsert = std::numeric_limits<double>::max();
        bool inserted = false;
        // Helper variables for the scheduled line selection
        int bestTwDepotStart = -1, bestTwDepotEnd = -1;
        // Iterate over each depot
        for (unsigned short i = 0; i < d_inst.d_nDepots; i++) {
            // Check if already inserted
            if (inserted) {
                break;
            }
            // Set helper variables for the scheduled line selection
            bool found = false;
            unsigned short SL = max;
            int twDepotStart = req.d_twDepotStart;
            int twDepotEnd = req.d_twDepotEnd;
            // Check if the request needs a scheduled line
            if (i!=req.d_idDepot) {
                d_tempSol.selectSL(SL,found,twDepotStart,twDepotEnd,req,i);
                // If no scheduled line is found, skip the depot
                if (!found) {
                    continue;
                }
            }
            // Iterate over each vehicle type
            for (unsigned short j = 0; j < d_tempSol.d_depotSols[i].d_nVehicleTypes; j++) {
                // Check if already inserted
                if (inserted) {
                    break;
                }
                // Iterate over each open route
                for (unsigned short k = 0; k < d_tempSol.d_depotSols[i].d_vehTypeSols[j].d_nUsedVehicles; k++) {
                    // Create a reference to the route
                    Route &route = d_tempSol.d_depotSols[i].d_vehTypeSols[j].d_routes[k];

                    // Check if the departure time of the route is before the earliest possible departure time
                    if (route.d_timeDeparture < twDepotStart ||
                        route.d_timeArrival > twDepotEnd) {
                        continue;
                    }
                    // Check the route if there are improved insertion options
                    std::pair<unsigned short, double> res = route.checkGreedyInsert(req, twDepotStart, 
                                                                                    twDepotEnd, d_graph,
                                                                                    d_inst.d_vehTypes[j]);
                    // If there is an improved insertion option, update the best insertion option
                    if (res.first == max) {
                        continue;
                    }
                    // Calculate noise if necessary
                    double costNoise = 0;
                    double costInsertTemp = res.second;
                    if (noise) {
                        costNoise = std::uniform_real_distribution<double>(-1,1)(d_gen)*d_settings.d_noiseLevel*costInsertTemp;
                    }
                    // Calculate reduced cost
                    if (SL != max) {
                        costInsertTemp = res.second + (d_inst.d_scheduledLines[SL].d_costRequest - d_duals.d_sl[SL]) *
                                                      static_cast<double>(req.d_qCol+req.d_qDel);
                    }
                    costInsertTemp -= d_duals.d_customer[req.d_idNode - d_duals.d_nDepots];
                    // If the resulting negative reduced cost is negative, store the label
                    if (route.d_cost + costInsertTemp < -d_settings.d_tolerance) {
                        storeLabel(i,j,k,iReq,SL,res.first, route.d_cost + costInsertTemp);
                    }
                    // If the cost of the current insertion is better than the best insertion option, update the best insertion option
                    if (costInsertTemp < (costInsert + costNoise)) {
                        bestDepot = i;
                        bestSL = SL;
                        bestVehicle = j;
                        bestRoute = k;
                        bestPos = res.first;
                        bestTwDepotStart = twDepotStart;
                        bestTwDepotEnd = twDepotEnd;
                        costInsert = costInsertTemp;
                    }
                }     
            }
        }
        if (inserted) {
            continue;
        }
        // Check the costs for openening a new route
        double costOpen = std::numeric_limits<double>::max();
        unsigned short depotOpen = max, vehicleOpen = max, idSLOpen = max;
        int twDepotStartOpen = req.d_twDepotStart, twDepotEndOpen = req.d_twDepotEnd;
        // Pass the variables and solve in place
        d_tempSol.checkOpenRouteGreedy(req,d_duals,noise,depotOpen,vehicleOpen,twDepotStartOpen,twDepotEndOpen,idSLOpen,costOpen,d_gen);
        // If the cost of opening a new route is better than the best insertion option, update the best insertion option
        if (costOpen < -d_settings.d_tolerance) {
            storeLabel(depotOpen,vehicleOpen,std::numeric_limits<unsigned short>::max(),iReq,idSLOpen,std::numeric_limits<unsigned short>::max(),costOpen);
        }
        if (costOpen < costInsert) {
            bestDepot = depotOpen;
            bestVehicle = vehicleOpen;
            bestTwDepotStart = twDepotStartOpen;
            bestTwDepotEnd = twDepotEndOpen;
            bestSL = idSLOpen;
            costInsert = costOpen;
            bestRoute = std::numeric_limits<unsigned short>::max();
            bestPos = std::numeric_limits<unsigned short>::max();
        }
        // Add the request to the solution at the best position
        inserted = insertNode(req,bestPos,bestDepot,bestVehicle,bestRoute,bestSL,bestTwDepotStart,bestTwDepotEnd,costInsert);
        if (!inserted) {
            break;
        }
    }
    return;
}

void Metaheuristic::repairGreedyLocal(bool noise) {
    /*
    Inserts a request into the solution using a localized version of the greedy insertion heuristic
    Repair operator #0 (without noise) and #1 (with noise)
    */
    // Shuffle the unassigned requests
    std::shuffle(d_tempSol.d_reqUnassigned.begin(), d_tempSol.d_reqUnassigned.end(), d_gen);
    unsigned short max = std::numeric_limits<unsigned short>::max();
    // Loop over unassigned requests
    while (d_tempSol.d_reqUnassigned.size()> 0) {
        // select the first unassigned request
        unsigned short iReq = d_tempSol.d_reqUnassigned[0];
        const Node &req = d_inst.d_nodes[iReq];
        // Find the list of nearest nodes
        auto nearestNodes = d_graph.getNearestNodes(iReq);
        // Find the locations of the nearest nodes
        std::set<std::tuple<unsigned short, unsigned short, unsigned short>> routesNearestNodes;
        for (auto it = nearestNodes.first; it != nearestNodes.second; ++it) {
            // Check if the nearest node is in visitLoc
            if (d_tempSol.d_visitLocs.find(*it) == d_tempSol.d_visitLocs.end()) {
                continue;
            }
            routesNearestNodes.insert(d_tempSol.d_visitLocs[*it].getRoute());
        }
        // Find the unique routes of the nearest nodes
        std::vector<std::tuple<unsigned short, unsigned short, unsigned short>> routesRelevant(routesNearestNodes.begin(), routesNearestNodes.end());
        // find the best insertion position
        unsigned short bestDepot = max, bestSL = max, bestVehicle = max, bestRoute = max, bestPos = max;
        double costInsert = std::numeric_limits<double>::max();
        bool inserted = false;
        // Helper variables for the scheduled line selection
        int bestTwDepotStart = -1, bestTwDepotEnd = -1;
        // Iterate over relevant routes
        for (auto it = routesRelevant.begin(); it != routesRelevant.end(); it++) {
            // Get the route
            unsigned short idDepot = std::get<0>(*it);
            unsigned short idVehicleType = std::get<1>(*it);
            unsigned short idRoute = std::get<2>(*it);
            Route &route = d_tempSol.d_depotSols[idDepot].d_vehTypeSols[idVehicleType].d_routes[idRoute];
            // Set helper variables for the scheduled line selection
            bool found = false;
            unsigned short SL = max;
            int twDepotStart = req.d_twDepotStart;
            int twDepotEnd = req.d_twDepotEnd;
            // Check if the request needs a scheduled line
            if (idDepot!=req.d_idDepot) {
                d_tempSol.selectSL(SL,found,twDepotStart,twDepotEnd,req,idDepot);
                // If no scheduled line is found, skip the depot
                if (!found) {
                    continue;
                }
            }
            // Check if the departure time of the route is before the earliest possible departure time
            if (route.d_timeDeparture < req.d_twDepotStart || route.d_timeArrival > req.d_twDepotEnd) {
                continue;
            }
            // Check the route if there are improved insertion options
            std::pair<unsigned short, double> res = route.checkGreedyInsert(req, twDepotStart, 
                                                                            twDepotEnd, d_graph, 
                                                                            d_inst.d_vehTypes[idVehicleType]);
            // If there is an improved insertion option, update the best insertion option
            if (res.first == max) {
                continue;
            }
            // Add noise if necessary
            double costNoise = 0;
            double costInsertTemp = res.second;
            if (noise) {
                costNoise = std::uniform_real_distribution<double>(-1,1)(d_gen)*d_settings.d_noiseLevel*costInsertTemp;
            }
            // Calculate reduced cost
            if (SL != max) {
                costInsertTemp = res.second + (d_inst.d_scheduledLines[SL].d_costRequest - d_duals.d_sl[SL]) * 
                                              static_cast<double>(req.d_qCol+req.d_qDel);
            }
            costInsertTemp -= d_duals.d_customer[req.d_idNode - d_duals.d_nDepots];
            // If the resulting negative reduced cost is negative, store the label
            if (route.d_cost + costInsertTemp < 0) {
                storeLabel(idDepot,idVehicleType,idRoute,iReq,SL,res.first,route.d_cost + costInsertTemp);
            }
            // If the cost of the current insertion is better than the best insertion option, update the best insertion option
            if (costInsertTemp < (costInsert + costNoise)) {
                bestDepot = idDepot;
                bestSL = SL;
                bestVehicle = idVehicleType;
                bestRoute = idRoute;
                bestPos = res.first;
                bestTwDepotStart = twDepotStart;
                bestTwDepotEnd = twDepotEnd;
                costInsert = costInsertTemp;
            }
        }
        // If the request is inserted, continue with the next request
        if (inserted) {
            continue;
        }
        // Check the costs for openening a new route
        double costOpen = std::numeric_limits<double>::max();
        unsigned short depotOpen = max, vehicleOpen = max, idSLOpen = max;
        int twDepotStartOpen = req.d_twDepotStart, twDepotEndOpen = req.d_twDepotEnd;
        // Pass the variables and solve in place
        d_tempSol.checkOpenRouteGreedy(req,d_duals,noise,depotOpen,vehicleOpen,twDepotStartOpen,twDepotEndOpen,idSLOpen,costOpen,d_gen);
        // If the cost of opening a new route is better than the best insertion option, update the best insertion option
        if (costOpen < -d_settings.d_tolerance) {
            storeLabel(depotOpen,vehicleOpen,std::numeric_limits<unsigned short>::max(),iReq,idSLOpen,std::numeric_limits<unsigned short>::max(),costOpen);
        }
        if (costOpen < costInsert) {
            bestDepot = depotOpen;
            bestVehicle = vehicleOpen;
            bestTwDepotStart = twDepotStartOpen;
            bestTwDepotEnd = twDepotEndOpen;
            bestSL = idSLOpen;
            costInsert = costOpen;
            bestRoute = max;
            bestPos = max;
        }
        // Add the request to the solution at the best position
        inserted = insertNode(req,bestPos,bestDepot,bestVehicle,bestRoute,bestSL,bestTwDepotStart,bestTwDepotEnd,costInsert);
        if (!inserted) {
            break;
        }
    }   
}

void Metaheuristic::repairRandomFirstFeasible() {
    /* 
    Inserts in the first route that is feasible, 
    at a Greedy position. If no route is feasible, open a new route
    in a Greedy manner.
    This is repair opeprator #2
    */
    // Shuffle the unassigned requests
    std::shuffle(d_tempSol.d_reqUnassigned.begin(), d_tempSol.d_reqUnassigned.end(), d_gen); 
    while (d_tempSol.d_reqUnassigned.size()> 0) {
        // select the first unassigned request
        unsigned short iReq = d_tempSol.d_reqUnassigned[0];
        const Node &req = d_inst.d_nodes[iReq];

        // find the first insertion position
        bool found = false;
        unsigned short max = std::numeric_limits<unsigned short>::max();
        unsigned short foundDepot = max, foundSL = max, foundVehicle = max, foundRoute = max, foundPos = max;
        double costInsert = std::numeric_limits<double>::max();
        int foundTwDepotStart = req.d_twDepotStart, foundTwDepotEnd = req.d_twDepotEnd;

        std::vector<unsigned short> idDepots;
        // Create vector of all depots and shuffle randomly
        idDepots = std::vector<unsigned short>(d_inst.d_nDepots);
        std::iota(idDepots.begin(), idDepots.end(), static_cast<unsigned short>(0));
        std::shuffle(idDepots.begin(), idDepots.end(), d_gen);
        for (unsigned short i = 0; i < idDepots.size(); i++) {
            if (found) {
                break;
            }
            // Set helper variables for the scheduled line selection
            unsigned short idDepot = idDepots[i];
            bool foundReqSL = false;
            unsigned short SL = max;
            int twDepotStart = req.d_twDepotStart;
            int twDepotEnd = req.d_twDepotEnd;
            // Check if the request needs a scheduled line
            if (idDepot!=req.d_idDepot) {
                d_tempSol.selectSL(SL,foundReqSL,twDepotStart,twDepotEnd,req,idDepot);
                // If no scheduled line is found, skip the depot
                if (!foundReqSL) {
                    continue;
                }
            }
            // Create a vector of all vehicle types and shuffle randomly
            std::vector<unsigned short> idVehicleTypes(d_tempSol.d_depotSols[idDepot].d_nVehicleTypes);
            std::iota(idVehicleTypes.begin(), idVehicleTypes.end(), static_cast<unsigned short>(0));
            std::shuffle(idVehicleTypes.begin(), idVehicleTypes.end(), d_gen);
            // Iterate over each vehicle type
            for (unsigned short j = 0; j < d_tempSol.d_depotSols[idDepot].d_nVehicleTypes; j++) {
                if (found) {
                    break;
                }
                unsigned short idVehicleType = idVehicleTypes[j];
                // If there are no routes, continue
                if (d_tempSol.d_depotSols[idDepot].d_vehTypeSols[idVehicleType].d_nUsedVehicles == 0) {
                    continue;
                }
                // Create a vector of all routes and shuffle randomly
                std::vector<unsigned short> idRoutes(d_tempSol.d_depotSols[idDepot].d_vehTypeSols[idVehicleType].d_nUsedVehicles);
                std::iota(idRoutes.begin(), idRoutes.end(), static_cast<unsigned short>(0));
                std::shuffle(idRoutes.begin(), idRoutes.end(), d_gen);
                // Iterate over each open route
                for (unsigned short k = 0; 
                    k < d_tempSol.d_depotSols[idDepot].d_vehTypeSols[idVehicleType].d_nUsedVehicles;
                    k++) {
                    unsigned short idRoute = idRoutes[k];
                    Route &route = d_tempSol.d_depotSols[idDepot].d_vehTypeSols[idVehicleType].d_routes[idRoute];
                    // Check if the departure time of the route is before the earliest possible departure time
                    if (route.d_timeDeparture < twDepotStart ||
                        route.d_timeArrival > twDepotEnd) {
                        continue;
                    }
                    // Check the route if there are insertion options
                    std::pair<unsigned short, double> res = route.checkGreedyInsert(req, twDepotStart, twDepotEnd, d_graph, d_inst.d_vehTypes[idVehicleType]);
                    // If there is a feasible option, make the insertion
                    if (SL != max) {
                        res.second += (d_inst.d_scheduledLines[SL].d_costRequest - d_duals.d_sl[SL]) *
                                      static_cast<double>(req.d_qCol+req.d_qDel);
                    }
                    if (res.first != max) {
                        foundDepot = idDepot;
                        foundSL = SL;
                        foundVehicle = idVehicleType;
                        foundRoute = idRoute;
                        foundPos = res.first;
                        foundTwDepotStart = twDepotStart;
                        foundTwDepotEnd = twDepotEnd;
                        costInsert = res.second - d_duals.d_customer[req.d_idNode - d_duals.d_nDepots];
                        found = true;
                        // If the resulting negative reduced cost is negative, store the label
                        if (route.d_cost + costInsert < -d_settings.d_tolerance) {
                            storeLabel(idDepot,idVehicleType,idRoute,iReq,SL,res.first,route.d_cost + costInsert);
                        }
                        break;
                    }
                }     
            }
        }
        if (!found) {
            // Check the costs for openening a new route
            bool noise = false;
            d_tempSol.checkOpenRouteGreedy(req,d_duals,noise,foundDepot,foundVehicle,foundTwDepotStart,foundTwDepotEnd,foundSL,costInsert,d_gen);
        }
        // Add the request to the solution at the found position
        bool inserted = insertNode(req,foundPos,foundDepot,foundVehicle,foundRoute,foundSL,foundTwDepotStart,foundTwDepotEnd,costInsert);
        if (!inserted) {
            break;
        }

    }
    return;
}

bool Metaheuristic::insertNode(const Node &req, unsigned short pos, unsigned short &idDepot, 
                              unsigned short &idVehicleType, unsigned short &idRoute, unsigned short &idSL,
                              int twDepotStart, int twDepotEnd, double costInsert) 
{
    /*
    Inserts a node into the solution
    */
    bool inserted = d_tempSol.insertNode(req,pos,idDepot,idVehicleType,idRoute,idSL,twDepotStart,twDepotEnd,costInsert);

    if (!inserted && d_bestSol.d_depotSols.size() == 0) {
        std::cout << "WARNING: Node could not be inserted into the solution during initialization.\n";
        return false;
    }
    else if (!inserted) {
        if (d_settings.d_verbose) {
            std::cout << "WARNING: Node could not be inserted into the solution. Reverting to the previous solution...\n";
        }
        d_tempSol = d_bestSol;
        return false;
    }
    return true;
}