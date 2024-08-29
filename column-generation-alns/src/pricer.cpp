#include "pricer.hpp"

Pricer::Pricer(const LPSolution& sol, const Settings& settings, const Instance& inst, const Graph& graph): 
d_sol(sol), d_settings(settings), d_inst(inst), d_graph(graph), d_idDepot(65535), d_idVehType(65535)
{
    d_durations = std::vector<std::vector<int>>(d_inst.d_nNodes, std::vector<int>(d_inst.d_nNodes, 0));
    d_reducedCosts = std::vector<std::vector<double>>(d_inst.d_nNodes, std::vector<double>(d_inst.d_nNodes, 0.0));
}

void Pricer::initEmpty()
{
    /*
    Init all data empty
    */
    d_pathsUnprocessed = {};
    d_pathsProcessed = {};

    for (unsigned short i = 0; i < d_inst.d_nNodes; i++)
    {
        d_pathsUnprocessed.push_back({});
        d_pathsProcessed.push_back({});
    }
    
    d_pathsComplete = {};
    d_routesUnique = {};

    d_nColsFound = 0;
    d_reducedCostBest = 0;
    d_ctrNewBest = 0;
}

std::deque<Label> Pricer::getCompleteLabels()
{
    return d_pathsComplete;
}

void Pricer::solveExact(std::atomic<bool>& stopFlag, unsigned short nColsMax)
{
    /*
    Solve the current pricing pricer exact

    :param max_columns: stop after max_columns are found with negative reduced cost
    */

    if (d_idDepot == -1 || d_idVehType == -1)
    {
        throw std::runtime_error("Error: no sub problem set.");
    }

    initEmpty();

    bool keepExtending = true;

    Label startLabel = createStartLabel();
    d_pathsUnprocessed[0].push_back(startLabel);
    
    while (keepExtending)
    {
        std::vector<unsigned short> initialSizes;

        for (unsigned short n = 0; n < d_inst.d_nNodes; n++)
        {
            initialSizes.push_back(d_pathsUnprocessed[n].size());
        }

        for (unsigned short n = 0; n < d_inst.d_nNodes; n++)
        {
            assert(n >= 0 && n < d_pathsUnprocessed.size());

            unsigned short counter = 0;

            // Extend all non-dominated paths
            while ( counter < initialSizes[n])
            {
                Label fromLabel = d_pathsUnprocessed[n].front();
                d_pathsUnprocessed[n].pop_front();

                for (unsigned short i = d_inst.d_nDepots; i < d_inst.d_nNodes; i++)
                {
                    if (stopFlag.load())
                    {
                        return;
                    }
                    // Remove later, only select first element
                    unsigned short idCustomer = i - d_inst.d_nDepots;

                    assert(idCustomer >= 0 && idCustomer < d_inst.d_nCustNodes);

                    if (d_sol.d_duals.d_customer[idCustomer] < 0.0 + d_settings.d_tolerance)
                    {
                        continue; // skip cust extension, dual close to 0
                    }

                    if (std::find(fromLabel.d_visitedNodes.begin(), fromLabel.d_visitedNodes.end(), i) != fromLabel.d_visitedNodes.end())
                    {
                        continue; // skip cust extension, customer already visited
                    }

                    extendLabel(fromLabel, d_inst.d_nodes[i]);
                }
                counter++;
            }
        }

        // Try to make complete paths of the currently extended labels
        for (unsigned short n = d_inst.d_nDepots; n < d_inst.d_nNodes; n++)
        {
            for (unsigned short i = 0; i < d_pathsUnprocessed[n].size(); i++)
            {
                if (stopFlag.load())
                {
                    return;
                }
                Label &partialPath = d_pathsUnprocessed[n][i]; // Ref since still check if can become complete path
                
                assert(static_cast<unsigned short>(partialPath.d_idNode) >= 0 && partialPath.d_idNode < d_reducedCosts.size());
                assert(d_idDepot >= 0 && d_idDepot < d_reducedCosts[partialPath.d_idNode].size());

                double costReturn = static_cast<double>(
                    d_reducedCosts[partialPath.d_idNode][d_idDepot]
                );

                if (partialPath.d_cost + costReturn < -d_settings.d_tolerance)
                {
                    Label completePath = partialPath; // Now we copy since complete route found
                    int lastNode = completePath.d_idNode;

                    completePath.d_idNode = d_idDepot;
                    completePath.d_visitedNodes.push_back(d_idDepot);
                    completePath.d_cost += costReturn;
                    completePath.d_curRouteTime += d_durations[lastNode][d_idDepot];
                    d_pathsComplete.push_back(completePath);

                    //std::cout << "Complete path: " << completePath << std::endl;
                    d_nColsFound++;
                }
            }
        }
        // STOP if we parsed all labels
        keepExtending = false;

        for (unsigned short n = d_inst.d_nDepots; n < d_inst.d_nNodes; n++)
        {
            if (d_pathsUnprocessed[n].size() > 0)
            {
                keepExtending = true;
            }
        }

        if (d_nColsFound > nColsMax)
        {
            keepExtending = false;
        }

        if (stopFlag.load())
        {
            return;
        }
    }

    // Sort and keen max_columns best paths found
    std::sort(this->d_pathsComplete.begin(), this->d_pathsComplete.end(), compareReducedCost);

    if (this->d_pathsComplete.size() > nColsMax)
    {
        this->d_pathsComplete.erase(this->d_pathsComplete.begin() + nColsMax, this->d_pathsComplete.end());
    }

    return;
}

void Pricer::extendLabel(const Label &labelFrom, const Node &nodeTo)
{
    /*
    Extend from_label to the new customer node.

    If feasible: create new label, calculate partial costs and add to unprocessed_labels.
    */

    // Check beta level: Fill up to level    
    if (nodeTo.d_qDel > 0 && labelFrom.d_qtyCanPickup < d_inst.d_vehTypes[d_idVehType].d_capacity - d_inst.d_vehTypes[d_idVehType].d_beta)
    {
        return; // STOP: cannot deliver due to fill-up-to-level
    }

    // Check alpha level: Mixing level
    unsigned short canDeliverTemp = labelFrom.d_qtyCanDeliver;

    if ((labelFrom.d_qtyCanPickup == d_inst.d_vehTypes[d_idVehType].d_capacity) && (nodeTo.d_qCol > 0) && (labelFrom.d_qtyCanDeliver > d_inst.d_vehTypes[d_idVehType].d_alpha))
    {
        canDeliverTemp = d_inst.d_vehTypes[d_idVehType].d_alpha; // Reduce can deliver to mixing level (since we had first collection)           
    }

    // Calculate new can collect and can deliver
    int qtyCanPickupNew = labelFrom.d_qtyCanPickup - nodeTo.d_qCol;
    int qtyCanDeliverNew = std::min(canDeliverTemp - nodeTo.d_qDel, labelFrom.d_qtyCanPickup - nodeTo.d_qCol);

    if (qtyCanPickupNew < 0 || qtyCanDeliverNew < 0)
    {
        return; // STOP: no capacity left to pick or deliver this cust
    }

    // Extend current time window (short int's later ???) static_cast<int>
    int timeToNode = d_durations[labelFrom.d_idNode][nodeTo.d_idNode];
    int timeToDepot = d_durations[nodeTo.d_idNode][d_idDepot];

    int routeDepTimeEarliest = std::max(labelFrom.d_depTimeEarliest - labelFrom.d_curRouteTime, nodeTo.d_twDepotStart);

    int twExtendedEarliestDepTime = routeDepTimeEarliest + labelFrom.d_curRouteTime + timeToNode + nodeTo.d_timeServ;
    int twExtendedLatestDepTime = labelFrom.d_depTimeLatest + timeToNode + nodeTo.d_timeServ;

    if (nodeTo.d_twCustEnd < twExtendedEarliestDepTime)
    {
        return; // STOP: to late at customer
    }
    int depTimeEarliestNew = std::max(twExtendedEarliestDepTime, nodeTo.d_twCustStart + nodeTo.d_timeServ); // Calculate earliest departure time at node
    int arrTimeEarliestNew = depTimeEarliestNew + timeToDepot;                               // Calculate earliest arrival time at depot
    int routeTimeNew = labelFrom.d_curRouteTime + timeToNode + nodeTo.d_timeServ; // UPDATE: Calculate route duration until departure at node
    int driveTimeNew = labelFrom.d_curDriveTime + timeToNode;                        // Calculate driving duration

    if (driveTimeNew + timeToDepot > d_inst.d_vehTypes[d_idVehType].d_timeDriveMax)
    {
        return; // STOP: vehicle driving for to long 
    }

    // NO TW
    bool hasTwNew = false;
    int depTimeLatestDepotNew = labelFrom.d_depTimeLatestDepot;  // Route latest start time stays the same
    int depTimeLatest = depTimeEarliestNew;         // No more tw, leave as early as possible at next customer
    int twWait = depTimeEarliestNew - twExtendedLatestDepTime;

    // TW
    if (labelFrom.d_hasTw && nodeTo.d_twCustStart < twExtendedLatestDepTime - nodeTo.d_timeServ)
    {
        hasTwNew = true;
        twWait = 0;
        depTimeLatest = std::min(twExtendedLatestDepTime, nodeTo.d_twCustEnd); // Latest dep time at node, because why wait when does not have to
        depTimeLatestDepotNew = depTimeLatest - routeTimeNew;                             // Latest dep time at depot
    }

    routeTimeNew += twWait; // Add waiting time to route length

    if (routeTimeNew + timeToDepot > d_inst.d_vehTypes[d_idVehType].d_timeMax)
    {
        return; // STOP: total route duration is to long
    }

    assert(labelFrom.d_idNode >= 0 && labelFrom.d_idNode < d_reducedCosts.size());
    assert(nodeTo.d_idNode >= 0 && nodeTo.d_idNode < d_reducedCosts.size());

    double costRoutingNew = d_reducedCosts[labelFrom.d_idNode][nodeTo.d_idNode];
    double costNew = labelFrom.d_cost + costRoutingNew;

    // Add extended label to the right partial paths queue
    std::vector<unsigned short int> N = labelFrom.d_visitedNodes;
    N.push_back(nodeTo.d_idNode);
    
    /* Sls: we need to determine:
        0) Get previous sl costs
        1) Optimal starting time (highest negative reduced cost)
        2) Optimal sl plan at this departure time ( which sls does every shipment take (if any))
    */
    // START: Get previous scheduled line costs
    double oldSLCost = 0;

    for (unsigned char idSL : labelFrom.d_sls)
    {
        if (idSL != 255)
        {
            assert( idSL < d_sol.d_duals.d_sl.size() );
            oldSLCost += d_sol.d_duals.d_sl[idSL];
        }
    }
    // END: Get previous scheduled line costs

    // START: Get route start times
    int earliestRouteStart = std::max(static_cast<int>(0), depTimeLatestDepotNew - (depTimeLatest - depTimeEarliestNew));    
    
    std::vector<int> PossibleRouteStarts;

    //std::vector<int> possible_route_end_times; // temp
    if (hasTwNew)
    {
        // We depart as early as possible
        PossibleRouteStarts.push_back(earliestRouteStart);

        // We depart right after scheduled line arrives
        for (const ScheduledLine &sl : d_inst.d_scheduledLines)
        {
            if (sl.d_idTo == d_idDepot &&
                sl.d_timeArr > earliestRouteStart &&
                sl.d_timeArr < depTimeLatestDepotNew)
            {
                PossibleRouteStarts.push_back(sl.d_timeArr);
            }
        }
    }
    else
    {
        // We depart at fixed time since we wait at customer
        PossibleRouteStarts.push_back(depTimeLatestDepotNew);
    }
    // END: Get route start times

    // Get route tw based on order release / deadline of shipments that are at the correct depot
    int startTimeLimit = 0;
    int endTimeLimit = d_inst.d_maxTimeHorizon;

    for ( const unsigned short idNode : N)
    {
        if ( idNode < d_inst.d_nDepots)
        {
            continue; 
        }

        // Depot does not even matter since sl also takes time
        const Node &node = d_inst.d_nodes[idNode];

        if ( node.d_qDel > 0 && node.d_twDepotStart > startTimeLimit)
        {
            startTimeLimit = node.d_twDepotStart;
        }
        else if ( node.d_qCol > 0 && node.d_twDepotEnd < endTimeLimit)
        {
            endTimeLimit = node.d_twDepotEnd;
        }     
    }

    // START: solve sl problem
    bool anySLPlan = false;
    double bestSLCost = -9999999999;
    int bestStartTime = 0;

    std::vector<unsigned char> slPlan;
    std::vector<int> routeEndTimes; // temp

    for (const int depTime : PossibleRouteStarts)
    {
        int arrTime = depTime + routeTimeNew + timeToDepot;
        routeEndTimes.push_back(arrTime); // temp

        // Check if feasible with respect to shipments that do not take sl
        if ( depTime < startTimeLimit || arrTime > endTimeLimit )
        {
            continue; // Start and/or end time route are not feasible
        }

        // Get a sl_plan with cost for this start time
        double reducedCostDepTime = 0;
        bool feasDepTime = true;
        std::vector<unsigned char> slSelected;
        unsigned short idRoute = 0;

        std::vector<unsigned short> qSLs(d_inst.d_scheduledLines.size(), 0);

        for ( const unsigned short idNode : N )
        {
            if ( idNode < d_inst.d_nDepots )
            {
                idRoute++;
                continue; 
            }
            // Look for scheduled line for node
            assert( idNode < d_inst.d_nodes.size() );
            const Node &node = d_inst.d_nodes[idNode];

            if ( node.d_idDepot == d_idDepot )
            {
                slSelected.push_back( 255 ); // Dummy, uses no sl
            }
            else
            {
                // Look for feasible cheapest sl
                unsigned char bestSL = 255;
                double bestSLReducedCost = -99999999;
                
                bool foundSLPrev = false; // Exclude previous inf sls
                unsigned char SLPre = 225;
                if ( idRoute == N.size() - 1 )
                { // Newly added node:
                    foundSLPrev = true;   // No previous sl
                }
                else
                {
                    assert( idRoute - 1 >= 0 && static_cast<unsigned short>(idRoute - 1) < labelFrom.d_sls.size() );
                    SLPre = labelFrom.d_sls[idRoute - 1];
                }
                if ( node.d_qDel > 0 )
                {
                    // Get previous sl of node
                    for ( unsigned char idSL : d_slsToDepot[node.d_idDepot] )
                    {
                        if ( idSL == SLPre )
                        {
                            foundSLPrev = true;
                        }
                        if ( foundSLPrev )
                        {
                            assert( idSL < d_inst.d_scheduledLines.size() );
                            assert( idSL < d_sol.d_duals.d_sl.size() );

                            const ScheduledLine &sl = d_inst.d_scheduledLines[idSL];
                            if ( sl.d_timeDep >= node.d_twDepotStart && sl.d_timeArr <= depTime )
                            {
                                if ( qSLs[idSL] + node.d_qDel <= sl.d_capacity )
                                {
                                    if ( d_sol.d_duals.d_sl[idSL] > bestSLReducedCost )
                                    {
                                        bestSLReducedCost = d_sol.d_duals.d_sl[idSL];
                                        bestSL = idSL;
                                        break; // Found optimal sl since they are sorted
                                    }
                                }
                            }
                        }
                    }
                }
                if ( node.d_qCol > 0 )
                {
                    for ( unsigned char idSL : d_slsFromDepot[node.d_idDepot] )
                    {
                        if ( idSL == SLPre )
                        {
                            foundSLPrev = true;
                        }
                        if ( foundSLPrev )
                        {
                            assert( idSL < d_inst.d_scheduledLines.size() );
                            assert( idSL < d_sol.d_duals.d_sl.size() );

                            const ScheduledLine &sl = d_inst.d_scheduledLines[idSL];
                            if ( sl.d_timeDep >= arrTime && sl.d_timeArr <= node.d_twDepotEnd )
                            {
                                if ( qSLs[idSL] + node.d_qCol <= sl.d_capacity )
                                {
                                    if ( d_sol.d_duals.d_sl[idSL] > bestSLReducedCost )
                                    {
                                        bestSLReducedCost = d_sol.d_duals.d_sl[idSL];
                                        bestSL = idSL;
                                        break; // Found optimal sl since they are sorted
                                    }
                                }
                            }
                        }
                    }
                }
                // Check if we found feas sl for node
                if ( bestSL == 255 )
                {
                    feasDepTime = false;
                    break; // for-loop over nodes, this departure time is INF
                }
                else
                {
                    qSLs[bestSL] += node.d_qCol + node.d_qDel;
                    reducedCostDepTime += bestSLReducedCost * (node.d_qCol + node.d_qDel);
                    slSelected.push_back(bestSL);
                }
            }
            idRoute++;
        }
        if ( feasDepTime )
        { 
            anySLPlan = true;

            // Check if current solution cost is better than other departure times
            if ( reducedCostDepTime > bestSLCost )
            {
                bestStartTime = depTime;
                bestSLCost = reducedCostDepTime;
                slPlan = slSelected;
            }
        }
    }
    // END: solve sl problem

    if ( !anySLPlan )
    {
        return; // STOP: no feasible sl plan
    }
    assert( slPlan.size() == N.size() - 1 ); // If feas, this should hold

    costNew += oldSLCost;
    costNew -= bestSLCost; // Negative and negative is +

    // Check if we need to pay for a new scheduled line
    if (slPlan[slPlan.size() - 1] != 255)
    {
        double qtySL = nodeTo.d_qCol + nodeTo.d_qDel;
        costNew += qtySL * d_inst.d_scheduledLines[0].d_costRequest; // sl_plan[sl_plan.size() - 1]
    }
    
    // Here we copy data over into new label
    Label extendedLabel = Label(
        hasTwNew,
        nodeTo.d_idNode,
        d_idVehType,
        d_idDepot,
        N,
        static_cast<unsigned short>(qtyCanPickupNew),
        static_cast<unsigned short>(qtyCanDeliverNew),
        depTimeEarliestNew,
        depTimeLatest,
        depTimeLatestDepotNew,
        arrTimeEarliestNew,
        routeTimeNew,
        driveTimeNew,
        costNew,
        bestStartTime,
        slPlan     
    );

    if (!isLabelDominated(extendedLabel))
    {
        removeLabelsDominatedBy(extendedLabel);  // Check if it dominates other labels
    
        d_pathsUnprocessed[nodeTo.d_idNode].push_back(extendedLabel);
        d_pathsProcessed[nodeTo.d_idNode].push_front(extendedLabel); // Sort later
    }

    return;
}

void Pricer::checkCompleteRoute(const Label &path)
{
    /*
    Based on current partial path: check if going back to depot right now would lead to reduced cost < 0
     If so, calculate real cost of route and save it
    */
    double costReturn = d_reducedCosts[path.d_idNode][d_idDepot];
    double reducedCost = path.d_cost + costReturn;

    if (reducedCost < -d_settings.d_tolerance)
    {
        if (reducedCost < this->d_reducedCostBest)
        {
            this->d_ctrNewBest++;
            this->d_reducedCostBest = reducedCost;
        }

        auto resPair = this->d_routesUnique.insert(path.d_visitedNodes);

        if (resPair.second)
        { // Insert succesfull => new route
            Label completePath = path; // Now we copy since complete route found
            unsigned short last_customer_node = completePath.d_idNode;

            completePath.d_idNode = d_idDepot;
            completePath.d_visitedNodes.push_back(d_idDepot);
            completePath.d_cost += reducedCost;
            completePath.d_curRouteTime += d_durations[last_customer_node][d_idDepot];

            this->d_pathsComplete.push_back(completePath);

            d_nColsFound++;
            //std::cout << "Complete path: " << completePath << std::endl;
        } 
    }
}

Label Pricer::createStartLabel()
{
    /*
    Create initial start label, start with reduced cost eta_sk
    */

    unsigned short vehCapacity = d_inst.d_vehTypes[d_idVehType].d_capacity;
    int timeHorMax = d_inst.d_maxTimeHorizon;
    unsigned short idDepot = d_idDepot;
    double fixedCost = d_inst.d_vehTypes[d_idVehType].d_costFixed;

    Label startLabel = Label(true, idDepot, d_idVehType, d_idDepot, {idDepot}, vehCapacity, vehCapacity, 0, timeHorMax, timeHorMax, 0, 0, 0, fixedCost, 0, {255});

    return startLabel;
}

void Pricer::setSubProblem(unsigned short idDepot, unsigned short idVehType)
{
    /*
    Set parameters and graph for the current sub-problem
    */
    // empty labels

    d_idDepot = idDepot;

    if (d_idVehType != idVehType)
    {
        d_idVehType = idVehType;
        updateDurationsMatrix();
    }

    updateCostMatrix();

    processAvailableSLs();

    return;
}

void Pricer::updateCostMatrix()
{
    /*
    Update cost matrix based on new duals

    validate
    */

    for (unsigned short i = 0; i < d_inst.d_nNodes; i++)
    {
        for (unsigned short j = 0; j < d_inst.d_nNodes; j++)
        {
            d_reducedCosts[i][j] = d_graph.getDist(d_idVehType, i, j) * d_inst.d_vehTypes[d_idVehType].d_costVariable;

            if (j >= d_inst.d_nDepots)
            {
                unsigned short idCustomer = j - d_inst.d_nDepots;

                assert(idCustomer >= 0 && idCustomer < d_sol.d_duals.d_customer.size());

                d_reducedCosts[i][j] -= d_sol.d_duals.d_customer[idCustomer];
            }

            if (i < d_inst.d_nDepots)
            {
                unsigned short idDepot = i;
                unsigned short idDual = idDepot * d_inst.d_nVehTypes + d_idVehType;

                assert(idDual < d_sol.d_duals.d_vehicle.size());

                d_reducedCosts[i][j] -= d_sol.d_duals.d_vehicle[idDual];
            }
        }
    }

    

    return;
}

void Pricer::updateDurationsMatrix()
{
    /*
    Update durations matrix when vehicle type changes since they have different speeds
    */
    for (unsigned short i = 0; i < d_inst.d_nNodes; i++)
    {
        for (unsigned short j = 0; j < d_inst.d_nNodes; j++)
        {
            d_durations[i][j] = d_graph.getTime(d_idVehType, i, j);
        }
    }

    return;
}

void Pricer::processAvailableSLs()
{
    /*
    Pre-process from and to SLs based on the selected -1and the duals
    */

    for (unsigned short idDepot = 0; idDepot < d_inst.d_nDepots; idDepot++)
    {
        if ( idDepot == d_idDepot )
        { 
            d_slsToDepot.push_back( {255} );
            d_slsFromDepot.push_back( {255} );

            continue; 
        }

        // Temp vector so we can sort on reduced cost
        std::vector<std::pair<unsigned char, double>> slsToTemp;
        std::vector<std::pair<unsigned char, double>> slsFromTemp;

        for (unsigned char idSL = 0; idSL < d_inst.d_nSL; idSL++)
        {
            assert( idSL < d_inst.d_scheduledLines.size() );
            assert( idSL < d_sol.d_duals.d_sl.size() );

            ScheduledLine sl = d_inst.d_scheduledLines[idSL];

            // Add to_sl
            if ( sl.d_idFrom == idDepot && sl.d_idTo == d_idDepot )
            {
                double dualSL = d_sol.d_duals.d_sl[idSL];
                std::pair<unsigned char, double> sl_pair(idSL, dualSL);
                slsToTemp.push_back(sl_pair);
            }

            // Add from_sl
            if ( sl.d_idFrom == d_idDepot && sl.d_idTo == idDepot )
            {
                double dualSL = d_sol.d_duals.d_sl[idSL];
                std::pair<unsigned char, double> sl_pair(idSL, dualSL);
                slsFromTemp.push_back(sl_pair);
            }
        }

        // Sort and extract sls from pairs
        std::sort(slsToTemp.begin(), slsToTemp.end(), [](
            const std::pair<unsigned char, double> &left, 
            const std::pair<unsigned char, double> &right
            )
            { return left.second > right.second; }
        );
        std::sort(slsFromTemp.begin(), slsFromTemp.end(), [](
            const std::pair<unsigned char, double> &left, 
            const std::pair<unsigned char, double> &right
            )
            { return left.second > right.second; }
        );

        std::vector<unsigned char> slsTo;
        std::vector<unsigned char> slsFrom;

        for ( std::pair<unsigned char, double> sl_pair2 : slsToTemp )
        {
            slsTo.push_back(sl_pair2.first);
        }

        for ( std::pair<unsigned char, double> sl_pair2 : slsFromTemp )
        {
            slsFrom.push_back(sl_pair2.first);
        }

        // Save scheduled lines (index idDepot is empty vector)
        d_slsToDepot.push_back(slsTo);
        d_slsFromDepot.push_back(slsFrom);
    }
    return;
}