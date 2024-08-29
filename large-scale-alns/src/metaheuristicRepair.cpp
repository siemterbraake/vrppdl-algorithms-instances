#include "metaheuristic.h"

void Metaheuristic::repairGreedy(bool noise) {
    /* 
    Repairs a solution using greedy insert
    Repair operator #0 (without noise) and #1 (with noise)
    */
    // Shuffle the unassigned requests
    std::shuffle(d_tempSol.d_reqUnassigned.begin(), d_tempSol.d_reqUnassigned.end(), d_gen);
    while (d_tempSol.d_reqUnassigned.size()> 0) {
        // select the first unassigned request
        std::size_t iReq = d_tempSol.d_reqUnassigned[0];
        Node &req = d_inst.d_nodes[iReq];

        // find the best insertion position
        std::size_t max = std::numeric_limits<std::size_t>::max();
        std::size_t bestDepot = max, bestSL = max, bestVehicle = max, bestRoute = max, bestPos = max;
        float costInsert = std::numeric_limits<float>::max();
        // Helper variables for the scheduled line selection
        int bestTwDepotStart = -1, bestTwDepotEnd = -1;
        // Iterate over each depot
        for (std::size_t i = 0; i < d_inst.d_nDepots; i++) {
            // Set helper variables for the scheduled line selection
            bool found = false;
            std::size_t SL = max;
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
            for (std::size_t j = 0; j < d_tempSol.d_depotSols[i].d_nVehicleTypes; j++) {
                // Iterate over each open route
                for (std::size_t k = 0; k < d_tempSol.d_depotSols[i].d_vehTypeSols[j].d_nUsedVehicles; k++) {
                    // Check if the departure time of the route is before the earliest possible departure time
                    if (d_tempSol.d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_timeDeparture < twDepotStart ||
                        d_tempSol.d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_timeArrival > twDepotEnd) {
                        continue;
                    }
                    // Check the route if there are improved insertion options
                    std::pair<std::size_t, float> res = d_tempSol.d_depotSols[i].d_vehTypeSols[j].d_routes[k].checkGreedyInsert(req, twDepotStart, 
                                                                                                                  twDepotEnd, d_graph,
                                                                                                                  d_inst.d_vehTypes[j]);
                    // If there is an improved insertion option, update the best insertion option
                    if (res.first == max) {
                        continue;
                    }
                    float costInsertTemp = res.second;
                    if (SL != max) {
                        costInsertTemp = res.second + d_inst.d_scheduledLines[SL].d_costRequest*static_cast<float>(req.d_qCol+req.d_qDel);
                    }
                    // Add noise if necessary
                    float costNoise = 0;
                    if (noise) {
                        costNoise = std::uniform_real_distribution<float>(-1,1)(d_gen)*d_settings.d_noiseLevel*costInsertTemp;
                    }
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
        // Check the costs for openening a new route
        float costOpen = std::numeric_limits<float>::max();
        std::size_t depotOpen = max, vehicleOpen = max, idSLOpen = max;
        int twDepotStartOpen = req.d_twDepotStart, twDepotEndOpen = req.d_twDepotEnd;
        // Pass the variables and solve in place
        d_tempSol.checkOpenRouteGreedy(req,noise,false,depotOpen,vehicleOpen,twDepotStartOpen,twDepotEndOpen,idSLOpen,costOpen,d_gen);
        // If the cost of opening a new route is better than the best insertion option, update the best insertion option
        if (costOpen < costInsert) {
            bestDepot = depotOpen;
            bestVehicle = vehicleOpen;
            bestTwDepotStart = twDepotStartOpen;
            bestTwDepotEnd = twDepotEndOpen;
            bestSL = idSLOpen;
            costInsert = costOpen;
            bestRoute = std::numeric_limits<std::size_t>::max();
            bestPos = std::numeric_limits<std::size_t>::max();
        }
        // Add the request to the solution at the best position
        d_tempSol.insertNode(req,bestPos,bestDepot,bestVehicle,bestRoute,
                   bestSL,bestTwDepotStart,bestTwDepotEnd,costInsert);
    }
    return;
}

void Metaheuristic::repairGreedyLocal(bool noise, bool restrictServiceArea) {
    /*
    Inserts a request into the solution using a localized version of the greedy insertion heuristic
    Repair operator #0 (without noise) and #1 (with noise)
    */
    // Shuffle the unassigned requests
    std::shuffle(d_tempSol.d_reqUnassigned.begin(), d_tempSol.d_reqUnassigned.end(), d_gen);
    std::size_t max = std::numeric_limits<std::size_t>::max();
    // Loop over unassigned requests
    while (d_tempSol.d_reqUnassigned.size()> 0) {
        // select the first unassigned request
        std::size_t iReq = d_tempSol.d_reqUnassigned[0];
        Node &req = d_inst.d_nodes[iReq];
        // Find the list of nearest nodes
        auto nearestNodes = d_graph.getNearestNodes(iReq);
        // Find the locations of the nearest nodes
        std::set<std::tuple<std::size_t, std::size_t, std::size_t>> routesNearestNodes;
        for (auto it = nearestNodes.first; it != nearestNodes.second; ++it) {
            // Check if the nearest node is in visitLoc
            if (d_tempSol.d_visitLocs.find(*it) == d_tempSol.d_visitLocs.end()) {
                continue;
            }
            if (!restrictServiceArea) {
                routesNearestNodes.insert(d_tempSol.d_visitLocs[*it].getRoute());
            }
            else {
                // Check if the nearest node is in the service area
                std::tuple<std::size_t, std::size_t, std::size_t> loc = d_tempSol.d_visitLocs[*it].getRoute();
                if (std::get<0>(loc) == req.d_idDepotAlloc) {
                    routesNearestNodes.insert(d_tempSol.d_visitLocs[*it].getRoute());
                }
            }
        }
        // Find the unique routes of the nearest nodes
        std::vector<std::tuple<std::size_t, std::size_t, std::size_t>> routesRelevant(routesNearestNodes.begin(), routesNearestNodes.end());
        // find the best insertion position
        std::size_t bestDepot = max, bestSL = max, bestVehicle = max, bestRoute = max, bestPos = max;
        float costInsert = std::numeric_limits<float>::max();
        // Helper variables for the scheduled line selection
        int bestTwDepotStart = -1, bestTwDepotEnd = -1;
        // Iterate over relevant routes
        for (auto it = routesRelevant.begin(); it != routesRelevant.end(); it++) {
            // Get the route
            std::size_t idDepot = std::get<0>(*it);
            std::size_t idVehicleType = std::get<1>(*it);
            std::size_t idRoute = std::get<2>(*it);
            // Set helper variables for the scheduled line selection
            bool found = false;
            std::size_t SL = max;
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
            if (d_tempSol.d_depotSols[idDepot].d_vehTypeSols[idVehicleType].d_routes[idRoute].d_timeDeparture < req.d_twDepotStart ||
                d_tempSol.d_depotSols[idDepot].d_vehTypeSols[idVehicleType].d_routes[idRoute].d_timeArrival > req.d_twDepotEnd) {
                continue;
            }
            // Check the route if there are improved insertion options
            std::pair<std::size_t, float> res = d_tempSol.d_depotSols[idDepot].d_vehTypeSols[idVehicleType].d_routes[idRoute].checkGreedyInsert(req, twDepotStart, 
                                                                                                                                 twDepotEnd, d_graph, 
                                                                                                                                 d_inst.d_vehTypes[idVehicleType]);
            // If there is an improved insertion option, update the best insertion option
            if (res.first == max) {
                continue;
            }
            float costInsertTemp = res.second;
            if (SL != max) {
                costInsertTemp = res.second + d_inst.d_scheduledLines[SL].d_costRequest*static_cast<float>(req.d_qCol+req.d_qDel);
            }
            // Add noise if necessary
            float costNoise = 0;
            if (noise) {
                costNoise = std::uniform_real_distribution<float>(-1,1)(d_gen)*d_settings.d_noiseLevel*costInsertTemp;
            }
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
        // Check the costs for openening a new route
        float costOpen = std::numeric_limits<float>::max();
        std::size_t depotOpen = max, vehicleOpen = max, idSLOpen = max;
        int twDepotStartOpen = req.d_twDepotStart, twDepotEndOpen = req.d_twDepotEnd;
        // Pass the variables and solve in place
        d_tempSol.checkOpenRouteGreedy(req,noise,restrictServiceArea,depotOpen,vehicleOpen,twDepotStartOpen,twDepotEndOpen,idSLOpen,costOpen,d_gen);
        // If the cost of opening a new route is better than the best insertion option, update the best insertion option
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
        d_tempSol.insertNode(req,bestPos,bestDepot,bestVehicle,bestRoute,
                   bestSL,bestTwDepotStart,bestTwDepotEnd,costInsert);
    }   
}

void Metaheuristic::repairRandomFirstFeasible(bool restrictServiceArea) {
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
        std::size_t iReq = d_tempSol.d_reqUnassigned[0];
        Node &req = d_inst.d_nodes[iReq];

        // find the first insertion position
        bool found = false;
        std::size_t max = std::numeric_limits<std::size_t>::max();
        std::size_t foundDepot = max, foundSL = max, foundVehicle = max, foundRoute = max, foundPos = max;
        float costInsert = std::numeric_limits<float>::max();
        int foundTwDepotStart = req.d_twDepotStart, foundTwDepotEnd = req.d_twDepotEnd;

        std::vector<std::size_t> idDepots;
        if (!restrictServiceArea) {
            // Create vector of all depots and shuffle randomly
            idDepots = std::vector<std::size_t>(d_inst.d_nDepots);
            std::iota(idDepots.begin(), idDepots.end(), static_cast<std::size_t>(0));
            std::shuffle(idDepots.begin(), idDepots.end(), d_gen);
        }
        else {
            idDepots = {req.d_idDepotAlloc};
        }
        for (std::size_t i = 0; i < idDepots.size(); i++) {
            if (found) {
                break;
            }
            // Set helper variables for the scheduled line selection
            std::size_t idDepot = idDepots[i];
            bool foundReqSL = false;
            std::size_t SL = max;
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
            std::vector<std::size_t> idVehicleTypes(d_tempSol.d_depotSols[idDepot].d_nVehicleTypes);
            std::iota(idVehicleTypes.begin(), idVehicleTypes.end(), static_cast<std::size_t>(0));
            std::shuffle(idVehicleTypes.begin(), idVehicleTypes.end(), d_gen);
            // Iterate over each vehicle type
            for (std::size_t j = 0; j < d_tempSol.d_depotSols[idDepot].d_nVehicleTypes; j++) {
                if (found) {
                    break;
                }
                std::size_t idVehicleType = idVehicleTypes[j];
                // If there are no routes, continue
                if (d_tempSol.d_depotSols[idDepot].d_vehTypeSols[idVehicleType].d_nUsedVehicles == 0) {
                    continue;
                }
                // Create a vector of all routes and shuffle randomly
                std::vector<std::size_t> idRoutes(d_tempSol.d_depotSols[idDepot].d_vehTypeSols[idVehicleType].d_nUsedVehicles);
                std::iota(idRoutes.begin(), idRoutes.end(), static_cast<std::size_t>(0));
                std::shuffle(idRoutes.begin(), idRoutes.end(), d_gen);
                // Iterate over each open route
                for (std::size_t k = 0; 
                    k < d_tempSol.d_depotSols[idDepot].d_vehTypeSols[idVehicleType].d_nUsedVehicles;
                    k++) {
                    std::size_t idRoute = idRoutes[k];
                    // Check if the departure time of the route is before the earliest possible departure time
                    if (d_tempSol.d_depotSols[idDepot].d_vehTypeSols[idVehicleType].d_routes[idRoute].d_timeDeparture < twDepotStart ||
                        d_tempSol.d_depotSols[idDepot].d_vehTypeSols[idVehicleType].d_routes[idRoute].d_timeArrival > twDepotEnd) {
                        continue;
                    }
                    // Check the route if there are insertion options
                    std::pair<std::size_t, float> res = d_tempSol.d_depotSols[idDepot].d_vehTypeSols[idVehicleType].d_routes[idRoute].checkGreedyInsert(req, twDepotStart, twDepotEnd, 
                                                                                                                                          d_graph, d_inst.d_vehTypes[idVehicleType]);
                    // If there is an improved insertion option, update the best insertion option
                    if (SL != max) {
                        res.second += d_inst.d_scheduledLines[SL].d_costRequest*static_cast<float>(req.d_qCol+req.d_qDel);
                    }
                    if (res.first != max) {
                        foundDepot = idDepot;
                        foundSL = SL;
                        foundVehicle = idVehicleType;
                        foundRoute = idRoute;
                        foundPos = res.first;
                        foundTwDepotStart = twDepotStart;
                        foundTwDepotEnd = twDepotEnd;
                        costInsert = res.second;
                        found = true;
                        break;
                    }
                }     
            }
        }
        if (!found) {
            // Check the costs for openening a new route
            bool noise = false;
            d_tempSol.checkOpenRouteGreedy(req,noise,restrictServiceArea,foundDepot,foundVehicle,foundTwDepotStart,foundTwDepotEnd,foundSL,costInsert,d_gen);
        }
        // Add the request to the solution at the found position
        d_tempSol.insertNode(req,foundPos,foundDepot,foundVehicle,foundRoute,
                   foundSL,foundTwDepotStart,foundTwDepotEnd,costInsert);
    }
    return;
}