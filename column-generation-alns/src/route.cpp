#include "route.hpp"

Route::Route() {
    /*
    Empty constructor for the route class
    */
    d_idRoute = std::numeric_limits<unsigned short>::max();
    d_idVehicleType = std::numeric_limits<unsigned short>::max();
    d_idDepot = std::numeric_limits<unsigned short>::max();
    d_nVisits = 0;
    d_timeDeparture = 0;
    d_timeArrival = 0;
    d_timeDrive = 0;
    d_timeTravel = 0;
    d_cost = 0;
    d_dist = 0;
    d_loadMax = 0;
    d_iMix = 0;
    d_iFill = 0;
}

Route::Route(unsigned short idRoute, unsigned short idVehicleType, unsigned short idDepot) : 
          d_idRoute(idRoute), d_idVehicleType(idVehicleType), d_idDepot(idDepot) {
    /*
    Constructor for the route class
    */
    d_nVisits = 0;
    d_timeDeparture = 0;
    d_timeArrival = 0;
    d_timeDrive = 0;
    d_timeTravel = 0;
    d_cost = 0;
    d_dist = 0;
    d_loadMax = 0;
    d_iMix = 0;
    d_iFill = 0;
}

std::pair<unsigned short,double> Route::checkGreedyInsert(const Node &req, int twDepotStart,
                                                int twDepotEnd, const Graph &graph, 
                                                const VehicleType &vehType) const {
    /*
    Takes a new customer node and determines the cheapest insert location and cost
    */
    double costInsert = std::numeric_limits<double>::max();
    unsigned short bestPos = std::numeric_limits<unsigned short>::max();
    if (req.d_type != 1 && req.d_type != 0) {
        std::cout << "WARNING: Request type for insertion is either depot or invalid, stopping..." << std::endl;
        return std::make_pair(bestPos,costInsert);
    }
    // Make sure the route departs after the request becomes available and the route
    // returns before it needs to move on in the system
    if (d_timeDeparture < twDepotStart) {
        return std::make_pair(bestPos,costInsert);
    }
    if (d_timeArrival > twDepotEnd) {
        return std::make_pair(bestPos,costInsert);
    }
    // Check if the capacity can be exceeded somewhere in the route due to this request
    bool checkCapacity = req.d_qDel+req.d_qCol+d_loadMax > std::min(vehType.d_alpha,vehType.d_beta);
    // Base case where request is a collection
    unsigned short iStart = d_iMix + 1, iEnd = d_nVisits-1;
    if (req.d_type == 1) {
        // Case where the request is a delivery
        iStart = 1;
        iEnd = d_iFill;
    }
    // Check if the capacity is exceeded at the start or end due to this request
    if (checkCapacity) {
        if (d_visits[d_nVisits-1].d_qCol+req.d_qCol > vehType.d_capacity) {
            // Capacity is exceeded at the end, skip this position
            return std::make_pair(bestPos,costInsert);
        } 
        if (d_visits[0].d_qDel+req.d_qDel > vehType.d_capacity) {
            // Capacity is exceeded at the start, skip this position
            return std::make_pair(bestPos,costInsert);
        }
    }
    // Check each feasible, first position is the same as the last position as the positions are in a loop
    for (unsigned short i=iStart;i<=iEnd;i++) {
        // Calculate the cost of the insert
        double distToReq = graph.getDist(vehType.d_idVehicleType,d_visits[i-1].d_idNode,req.d_idNode);
        double distFromReq = graph.getDist(vehType.d_idVehicleType,req.d_idNode,d_visits[i].d_idNode);
        double distInsert = distToReq + distFromReq - graph.getDist(vehType.d_idVehicleType,d_visits[i-1].d_idNode,d_visits[i].d_idNode);
        // if the position is not the best, continue
        if (distInsert*vehType.d_costVariable > costInsert) {
            continue;
        }
        // Otherwise, verify feasibility
        bool feasible = true;
        if (checkCapacity) {
            // Check if total vehicle capacity is exceeded at this position
            if (d_visits[i-1].d_qCol+d_visits[i-1].d_qDel+req.d_qDel+req.d_qCol > vehType.d_capacity) {
                // Capacity is exceeded, skip this position
                continue;
            }
            // Check if fill up to level is exceeded at this position
            if (req.d_type == 0 && d_visits[i-1].d_qCol + req.d_qCol > vehType.d_beta && d_visits[i-1].d_qDel > 0) {
                // Fill up to level is exceeded, skip this position
                continue;
            }
            // Check if mixing level is exceeded at this position
            if (req.d_type == 1 && d_visits[i].d_qDel + req.d_qDel > vehType.d_alpha && d_visits[i].d_qCol > 0) {
                // Mixing level is exceeded, skip this position
                continue;
            }
            if (req.d_type == 0) {
                // Case of a collection, check if capacity is exceeded later in the route
                for (unsigned short j=d_nVisits-1;j>=i;j--) {
                // Reverse iteration as violation is more likely at the end
                    if (d_visits[j].d_qDel+d_visits[j].d_qCol+req.d_qCol > vehType.d_capacity) {
                        // Capacity is exceeded, skip this position
                        feasible = false;
                        break;
                    }
                    if (d_visits[j].d_type == 1 && d_visits[j].d_qCol+req.d_qCol > vehType.d_beta) {
                        // Mixing level is exceeded, skip this position
                        feasible = false;
                        break;
                    }
                } 
            }
            // Check if feasible is still true, if not skip this position
            if (!feasible) {
                continue;
            }
            if (req.d_type == 1) {
                // Case where the request is a delivery, check if the capacity is not exceeded earlier in the route
                for (unsigned short j=1;j<i;j++) {
                    if (d_visits[j].d_qDel+d_visits[j].d_qCol+req.d_qDel > vehType.d_capacity) {
                        // Capacity is exceeded, skip this position
                        feasible = false;
                        break;
                    }
                    if (d_visits[j].d_type == 0 && d_visits[j].d_qDel+req.d_qDel > vehType.d_alpha) {
                        // Mixing level is exceeded, skip this position
                        feasible = false;
                        break;
                    }
                } 
            }
        }
        // Check if feasible is still true, if not skip this position
        if (!feasible) {
            continue;
        }
        // Check if the time window is violated at the start
        int timeToReq = graph.getTime(vehType.d_idVehicleType,d_visits[i-1].d_idNode,req.d_idNode);
        int timeFromReq = graph.getTime(vehType.d_idVehicleType,req.d_idNode,d_visits[i].d_idNode);
        int timeDriveInsert = timeToReq + timeFromReq - graph.getTime(vehType.d_idVehicleType,d_visits[i-1].d_idNode,d_visits[i].d_idNode);

        if (d_timeDrive+timeDriveInsert > vehType.d_timeDriveMax) {
            // Drive time limit is exceeded, skip this position
            continue;
        }
        int timeArrivalReq;
        // For the first insertion position, the arrival time may be affected by the release time 
        if (i == 1 && d_visits[0].d_timeDeparture < twDepotStart) {
            // If the departure time is before the depot time window, update the departure time
            timeArrivalReq = twDepotStart + timeToReq;
        }
        else {
            timeArrivalReq = d_visits[i-1].d_timeDeparture + timeToReq;
        }

        if (timeArrivalReq + req.d_timeServ > req.d_twCustEnd) {
            // Time window is violated at the end, skip this position
            continue;
        }
        if (timeArrivalReq < req.d_twCustStart) {
            // Wait at the request until the time window starts
            timeArrivalReq = req.d_twCustStart;
        }
        // Determine impact on future time windows
        int impact = timeArrivalReq + req.d_timeServ+timeFromReq - d_visits[i].d_timeArrival;
        int impactTimeTravel = 0;
        // When adding the last customer node, the travel time may be affected even when impact is negative
        if (i==d_nVisits-1) {
            int newTimeTravel =  timeArrivalReq + req.d_timeServ + timeFromReq - d_timeDeparture;
            if (newTimeTravel > vehType.d_timeMax) {
                // Total time limit is exceeded, skip this position
                continue;
            }  
            if (d_visits[d_nVisits-1].d_timeArrival > twDepotEnd && d_timeDeparture + newTimeTravel > twDepotEnd) {
                // Return time limit is exceeded, skip this position
                continue;
            }
        }

        // If impact is positive, the time window may be violated for future requests, so check
        if (impact > 0) {
            for (unsigned short j=i;j<d_nVisits-1;j++) {
                if (d_visits[j].d_twSlackEnd < impact) {
                    // Time window is violated for future requests, skip this position
                    feasible = false;
                    break;
                }
                // Store the impact at the last customer node
                if (j == d_nVisits-2) {
                    impactTimeTravel = impact;
                }
                // If you need to wait at the next node, update the impact
                impact -= d_visits[j+1].d_twWait;
            }
            // If this request pushes the route over the return time for scheduled lines, skip this position
            if (impact > 0) {
                continue;
            }
        }
        // Check if feasible is still true, if not skip this position
        if (!feasible) {
            continue;
        }

        // For the first position, check if the total time is exceeded due to a possible earlier departure
        if (i == 1) {
            impactTimeTravel = std::max(impactTimeTravel, 0);
            int newTimeTravel = d_visits[d_nVisits-1].d_timeArrival-d_visits[d_nVisits-1].d_twWait - (timeArrivalReq - timeToReq) + impactTimeTravel;
            if (newTimeTravel > vehType.d_timeMax) {
                // Total time limit is exceeded, skip this position
                continue;
            }
            if (timeArrivalReq - timeToReq + newTimeTravel > twDepotEnd) {
                // Return time limit is exceeded, skip this position
                continue;
            }
        }
        // If the required return time of the request is smaller than the current return time, check if this return time
        // is violated as well
        if (d_visits[d_nVisits-1].d_timeArrival > twDepotEnd) {
            int timeArrivalRoute;
            impactTimeTravel = std::max(impactTimeTravel, 0);
            timeArrivalRoute = d_timeDeparture + d_timeTravel + impactTimeTravel;
            if (timeArrivalRoute > twDepotEnd) {
                // Return time limit is exceeded, skip this position
                continue;
            }
        }

        // Check if total time is exceeded at this position
        if (d_timeTravel + impactTimeTravel > vehType.d_timeMax) {
            // Total time limit is exceeded, skip this position
            continue;
        }
        if (feasible) {
            costInsert = distInsert*vehType.d_costVariable;
            bestPos = i;
        }
    }
    return std::make_pair(bestPos,costInsert);
}

void Route::insert(const Node &req, unsigned short pos, int twDepotStart,
                   int twDepotEnd, const Graph &graph, const VehicleType &vehType, 
                   std::unordered_map<unsigned short, VisitLoc> &visitLocs) {
    /*
    Takes a new customer node and inserts it at the given position
    */
    // Update route parameters
    d_nVisits++;

    // Calculate new distance
    double distToReq = graph.getDist(vehType.d_idVehicleType,d_visits[pos-1].d_idNode,req.d_idNode);
    double distFromReq = graph.getDist(vehType.d_idVehicleType,req.d_idNode,d_visits[pos].d_idNode);
    double distRem = graph.getDist(vehType.d_idVehicleType,d_visits[pos-1].d_idNode,d_visits[pos].d_idNode);
    double distInsert = distToReq+distFromReq-distRem;
    // Calculate new time drive
    int timeToReq = graph.getTime(vehType.d_idVehicleType,d_visits[pos-1].d_idNode,req.d_idNode);
    int timeFromReq = graph.getTime(vehType.d_idVehicleType,req.d_idNode,d_visits[pos].d_idNode);
    int timeRem = graph.getTime(vehType.d_idVehicleType,d_visits[pos-1].d_idNode,d_visits[pos].d_idNode);
    int timeDriveInsert = timeToReq+timeFromReq-timeRem;

    d_timeDrive += timeDriveInsert;
    d_dist += distInsert;

    unsigned short qDel = 0, qCol = 0;

    if (req.d_type == 0) {
       // Case of a collection
        qDel = d_visits[pos-1].d_qDel;
        qCol = d_visits[pos-1].d_qCol + req.d_qCol;
        // Update the quantities for the rest of the route
        for (unsigned short i=pos;i<d_nVisits-1;i++) {
            // If it is a collection, only the collection quantity needs to be updated
            d_visits[i].d_qCol += req.d_qCol;
            if (d_visits[i].d_qDel+d_visits[i].d_qCol > d_loadMax) {
                d_loadMax = d_visits[i].d_qDel+d_visits[i].d_qCol;
            }
        }
    }
    else if (req.d_type == 1) {
        // Case of a delivery
        qDel = d_visits[pos-1].d_qDel;
        qCol = d_visits[pos-1].d_qCol;
        // Update the quantities for the start of the route
        unsigned short i = pos;
        while (i-- > 0) {
            // If it is a collection, only the collection quantity needs to be updated
            d_visits[i].d_qDel += req.d_qDel;
            if (d_visits[i].d_qDel+d_visits[i].d_qCol > d_loadMax) {
                d_loadMax = d_visits[i].d_qDel+d_visits[i].d_qCol;
            }
        }
        // iFill is increased by one as the delivery must be inserted before iFill
        d_iFill++;
    }
    // Determine if the departure time at the depot is affected
    if (d_visits[0].d_timeDeparture < twDepotStart) {
        // If the departure time is before the depot time window, update the departure time
        if (pos!=1) {
            d_visits[1].d_twWait -= twDepotStart-d_visits[0].d_timeDeparture;
        }
        d_visits[0].d_timeDeparture = twDepotStart;
    }
    // Determine arrival time at the request
    int timeArrivalReq = d_visits[pos-1].d_timeDeparture+timeToReq;
    int twWait = 0;

    if (timeArrivalReq < req.d_twCustStart) {
        // Wait at the request until the time window starts
        twWait = req.d_twCustStart-timeArrivalReq;
        timeArrivalReq = req.d_twCustStart;
    }
    int twSlackStart = timeArrivalReq-req.d_twCustStart;
    int twSlackEnd = req.d_twCustEnd-timeArrivalReq-req.d_timeServ;
    // Insert the new visit at the given position
    d_visits.insert(d_visits.begin()+static_cast<int>(pos),Visit(timeArrivalReq,timeArrivalReq+req.d_timeServ,twWait,twSlackStart,twSlackEnd,
                                           pos,req.d_idNode,qDel,qCol,req.d_type));
    // Update iFill and iMix
    updateLoadHelpers(vehType);
    // Detemine time shift for the rest of the route
    int timeShift = d_visits[pos].d_timeDeparture+timeFromReq-d_visits[pos+1].d_timeArrival;
    double updateTw = true;

    for (unsigned short i=pos+1;i<d_nVisits;i++) {
        d_visits[i].d_idVisit = i;
        // Update visitLoc hashmap if the visit is not the last one
        if (i<d_nVisits-1) {
        visitLocs[d_visits[i].d_idNode].d_idVisit = i;
        }
        if (updateTw) {
            // If the timeshift is zero or negative, this means the route must wait at the request
            // And the rest of the route is unaffected
            if (timeShift <= 0) {
                d_visits[i].d_twWait = -timeShift;
                updateTw = false;
                continue;
            }
            d_visits[i].d_timeArrival += timeShift;
            d_visits[i].d_timeDeparture += timeShift;
            d_visits[i].d_twSlackStart += timeShift;
            d_visits[i].d_twSlackEnd -= timeShift;
            d_visits[i].d_twWait = 0;
            timeShift = timeShift-d_visits[i+1].d_twWait;
        }
    }
    // Update travel time variables
    if (pos == 1) {
        d_timeDeparture = d_visits[1].d_timeArrival-timeToReq;
    }
    // Check if the arrival time at the last visit is affected
    if (d_visits[d_nVisits-1].d_timeArrival > twDepotEnd) {
        // If the arrival time is after the depot time window, update the arrival time
        d_visits[d_nVisits-1].d_twWait -= d_visits[d_nVisits-1].d_timeArrival-twDepotEnd;
        d_visits[d_nVisits-1].d_timeArrival = twDepotEnd;
    }
    d_timeArrival = d_visits[d_nVisits-1].d_timeArrival-d_visits[d_nVisits-1].d_twWait;
    d_timeTravel = d_timeArrival-d_timeDeparture;
    return;
}

void Route::remove(const Node &req, unsigned short pos, const Graph &graph, const VehicleType &vehType, 
                   const Instance &inst, std::unordered_map<unsigned short, VisitLoc> &visitLocs) {
    /*
    Takes a new position and removes it from the route
    */
    // Determine type of the visit to be removed
    short nodeType = d_visits[pos].d_type;
    // Determine removal cost
    double distToReq = graph.getDist(vehType.d_idVehicleType,d_visits[pos-1].d_idNode,d_visits[pos].d_idNode);
    double distFromReq = graph.getDist(vehType.d_idVehicleType,d_visits[pos].d_idNode,d_visits[pos+1].d_idNode);
    double distNewEdge = graph.getDist(vehType.d_idVehicleType,d_visits[pos-1].d_idNode,d_visits[pos+1].d_idNode);
    double distRemove = distToReq+distFromReq-distNewEdge;
    // Determine removal time
    int timeToReq = graph.getTime(vehType.d_idVehicleType,d_visits[pos-1].d_idNode,d_visits[pos].d_idNode);
    int timeFromReq = graph.getTime(vehType.d_idVehicleType,d_visits[pos].d_idNode,d_visits[pos+1].d_idNode);
    int timeNewEdge = graph.getTime(vehType.d_idVehicleType,d_visits[pos-1].d_idNode,d_visits[pos+1].d_idNode);
    int timeRemove = timeToReq+timeFromReq-timeNewEdge;
    // Update route parameters
    d_cost -= distRemove*vehType.d_costVariable;
    d_dist -= distRemove;
    d_timeDrive -= timeRemove;

    if (nodeType == 0) {
        // Update the quantities for the rest of the route
        for (unsigned short i=pos+1;i<d_nVisits;i++) {
            // If it is a collection, only the collection quantity needs to be updated
            d_visits[i].d_qCol -= req.d_qCol;
        }
    }
    else if (nodeType == 1) {
        for (unsigned short i=0;i<pos;i++) {
            // If it is a delivery, only the delivery quantity needs to be updated
            d_visits[i].d_qDel -= req.d_qDel;
        }
    }
    // Check if the departure time at the depot is affected
    if (req.d_type == 1 && d_visits[0].d_timeDeparture != 0) {
        // Determine min required departure time
        int timeMinDepartureReq;
        if (visitLocs.at(req.d_idNode).d_idSL != std::numeric_limits<unsigned short>::max()) {
            // If the request is on a scheduled line, the arrival time of the SL is the min required departure time
            timeMinDepartureReq = inst.d_scheduledLines[visitLocs.at(req.d_idNode).d_idSL].d_timeArr;
        }
        else {
            // If the depot is not a scheduled line, the min required departure time is the start of the depot window
            timeMinDepartureReq = inst.d_nodes[req.d_idNode].d_twDepotStart;
        }
        if (d_visits[0].d_timeDeparture == timeMinDepartureReq) {
            // If the departure time is equal to the min required departure time, relax this limit
            int timeMinDeparture = 0;
            for (unsigned short i=1;i<d_nVisits-1;i++) {
                if (i==pos || d_visits[i].d_type == 0) {
                    // Skip the removed position
                    continue;
                }
                if (visitLocs.at(d_visits[i].d_idNode).d_idSL != std::numeric_limits<unsigned short>::max()) {
                    // If the request is on a scheduled line, the arrival time of the SL is the min required departure time
                    timeMinDepartureReq = inst.d_scheduledLines[visitLocs.at(d_visits[i].d_idNode).d_idSL].d_timeArr;
                }
                else {
                    // If the depot is not a scheduled line, the min required departure time is the start of the depot window
                    timeMinDepartureReq = inst.d_nodes[d_visits[i].d_idNode].d_twDepotStart;
                }
                if (timeMinDepartureReq > timeMinDeparture) {
                    timeMinDeparture = timeMinDepartureReq;
                }
            }
            d_visits[1].d_twWait += d_visits[0].d_timeDeparture - timeMinDeparture;
            d_visits[0].d_timeDeparture = timeMinDeparture;
        }
    }
    // Check if the arrival time at the last visit is affected
    if (req.d_type == 0 && d_visits[d_nVisits-1].d_timeArrival != d_visits[d_nVisits-1].d_timeDeparture) {
        // Determine min required departure time
        int timeMaxArrivalReq;
        if (visitLocs.at(req.d_idNode).d_idSL != std::numeric_limits<unsigned short>::max()) {
            // If the request is on a scheduled line, the arrival time of the SL is the max required arrival time
            timeMaxArrivalReq = inst.d_scheduledLines[visitLocs.at(req.d_idNode).d_idSL].d_timeDep;
        }
        else if (req.d_idDepot >= inst.d_nDepots && req.d_twDepotEnd < d_visits[d_nVisits-1].d_timeDeparture) {
            // If the request is not pre-assigned to a depot, the max required arrival time is the end of the depot window
            timeMaxArrivalReq = inst.d_nodes[visitLocs.at(req.d_idNode).d_idDepot].d_twCustEnd;
        }
        else {
            // Otherwise, the max required arrival time is the end of the depot window
            timeMaxArrivalReq = inst.d_nodes[req.d_idNode].d_twDepotEnd;
        }
        if (d_visits[d_nVisits-1].d_timeArrival == timeMaxArrivalReq) {
            // If the arrival time is equal to the max required arrival time, relax this limit
            int timeMaxArrival = d_visits[d_nVisits-1].d_timeDeparture;
            for (unsigned short i=1;i<d_nVisits-1;i++) {
                if (i==pos || d_visits[i].d_type == 1) {
                    // Skip the removed position
                    continue;
                }
                // If the type is a collection, check if the timeMaxArrival is higher than the current value
                if (visitLocs.at(d_visits[i].d_idNode).d_idSL != std::numeric_limits<unsigned short>::max()) {
                    // If the request is on a scheduled line, the departure time of the SL is the min required departure time
                    timeMaxArrivalReq = inst.d_scheduledLines[visitLocs.at(d_visits[i].d_idNode).d_idSL].d_timeDep;
                }
                else if (inst.d_nodes[d_visits[i].d_idNode].d_idDepot >= inst.d_nDepots && inst.d_nodes[d_visits[i].d_idNode].d_twDepotEnd < d_visits[d_nVisits-1].d_timeDeparture) {
                    // If the request is not pre-assigned to a depot, the max required arrival time is the end of the depot window
                    timeMaxArrivalReq = inst.d_nodes[visitLocs.at(d_visits[i].d_idNode).d_idDepot].d_twCustEnd;
                }
                else {
                    // If the depot is not a scheduled line, the min required departure time is the start of the depot window
                    timeMaxArrivalReq = inst.d_nodes[d_visits[i].d_idNode].d_twDepotEnd;
                }
                if (timeMaxArrivalReq < timeMaxArrival) {
                    timeMaxArrival = timeMaxArrivalReq;
                }
            }
            d_visits[d_nVisits-1].d_twWait += timeMaxArrival - d_visits[d_nVisits-1].d_timeArrival;
            d_visits[d_nVisits-1].d_timeArrival = timeMaxArrival;
        }
    }
    // Update the time windows for the rest of the route
    int timeShift = d_visits[pos-1].d_timeDeparture+timeNewEdge-d_visits[pos+1].d_timeArrival;
    double moveTw = true;
    for (unsigned short i=pos+1;i<d_nVisits;i++) {
        d_visits[i].d_idVisit = i-1;
        int newTwWait = d_visits[i].d_twWait;
        // Update visitLoc hashmap if the visit is not the last one
        if (i<d_nVisits-1) {
            visitLocs[d_visits[i].d_idNode].d_idVisit = i-1;
        }
        // Make sure the shift remains within the time window
        if (-1*timeShift > d_visits[i].d_twSlackStart) {
            newTwWait = -1*timeShift-d_visits[i].d_twSlackStart;
            timeShift = -1*d_visits[i].d_twSlackStart;
            if (i > pos+1) {
                newTwWait += d_visits[i].d_twWait;
            }
        }
        // Update the waiting time 
        d_visits[i].d_twWait = newTwWait;      
        if (timeShift == 0) {
            moveTw = false;
        }
        if (moveTw) {
            // If the timeshift is negative, perform the timeshift
            d_visits[i].d_timeArrival += timeShift;
            d_visits[i].d_timeDeparture += timeShift;
            d_visits[i].d_twSlackStart += timeShift;
            d_visits[i].d_twSlackEnd -= timeShift;  
        }
    }
    // Remove the visit at the given position
    d_visits.erase(d_visits.begin()+static_cast<int>(pos));
    d_nVisits--;
    // Update and repair the route
    removeUnwantedSlack();
    // Update timeTravel
    if (pos==1) {
        d_timeDeparture = d_visits[1].d_timeArrival-timeNewEdge;
    }
    d_timeArrival = d_visits[d_nVisits-1].d_timeArrival-d_visits[d_nVisits-1].d_twWait;
    d_timeTravel = d_timeArrival-d_timeDeparture;
    // Update loadMax, iMix and iFill
    updateLoadHelpers(vehType);
}

void Route::removeUnwantedSlack() {
    /*
    Removes unwanted slack from the route
    */
    int timeShift = 0;
    for (unsigned short i=1;i<d_nVisits-1;i++) {
        // If both slack and wait are positive, the slack must be removed
        int newTwWait = 0;
        if (d_visits[i].d_twWait > 0 && d_visits[i].d_twSlackStart > 0) {
            timeShift += -1*std::min(d_visits[i].d_twWait,d_visits[i].d_twSlackStart);
            newTwWait = d_visits[i].d_twWait+timeShift;
        }
        if (timeShift < 0) {
            // Make sure the shift remains within the time window
            if (-1*timeShift > d_visits[i].d_twSlackStart) {
                newTwWait = -1*timeShift-d_visits[i].d_twSlackStart+d_visits[i].d_twWait;
                timeShift = -1*d_visits[i].d_twSlackStart;
            }
            // Update the waiting time
            d_visits[i].d_twWait = newTwWait;
            // perform the timeshift
            d_visits[i].d_timeArrival += timeShift;
            d_visits[i].d_timeDeparture += timeShift;
            d_visits[i].d_twSlackStart += timeShift;
            d_visits[i].d_twSlackEnd -= timeShift;
            // If the first position is moved, update the departure time
            if (i==1) {
                d_timeDeparture += timeShift;
            }
        }
    }
    if (timeShift != 0) {
        // Update timeTravel
        d_visits[d_nVisits-1].d_twWait -= timeShift;
    }
    return;
}

void Route::updateLoadHelpers(const VehicleType &vehType) {
    /* 
    Updates IMix, IFill and LoadMax for the route.
    */
    unsigned short newMaxLoad = 0;
    unsigned short newIFill = d_nVisits-1, newIMix = 1;
    bool iFillPassed = false, iMixPassed = false;
    for (unsigned short i=0;i<d_nVisits;i++) {
        if (d_visits[i].d_qDel+d_visits[i].d_qCol > newMaxLoad) {
            newMaxLoad = d_visits[i].d_qDel+d_visits[i].d_qCol;
        }
        if (!iFillPassed && d_visits[i].d_qCol > vehType.d_beta) {
            newIFill = i;
            iFillPassed = true;
        }
        if (!iMixPassed && d_visits[i].d_qDel < vehType.d_alpha) {
            newIMix = i;
            iMixPassed = true;
        }
    }
    d_loadMax = newMaxLoad;
    d_iFill = newIFill;
    d_iMix = std::max(newIMix,unsigned short(0));
}

Label Route::toLabel(const Instance& inst, const std::unordered_map<unsigned short, VisitLoc>& visitLocs) const {
    /* 
    Writes the route to a label
    */
    Label label;
    label.d_cost = d_cost;
    label.d_idVehType = d_idVehicleType;
    label.d_idDepot = d_idDepot;
    for (Visit visit : d_visits) {
        label.d_visitedNodes.push_back(visit.d_idNode);
        if (visit.d_idNode < inst.d_nDepots) {
            continue;
        }
        const VisitLoc &loc = visitLocs.at(visit.d_idNode);
        if (loc.d_idSL != std::numeric_limits<unsigned short>::max()) {
            label.d_sls.push_back(loc.d_idSL);
        }
        else {
            label.d_sls.push_back(255);
        }
    }
    return label;
}