#include "metaheuristic.hpp"

Metaheuristic::Metaheuristic(const Settings &settings, const Instance &inst, const Graph &graph, unsigned short seed):
d_settings(settings), d_inst(inst), d_graph(graph)
 {
    /*
    Constructor for the metaheuristic struct.
    */
    d_gen.seed(seed);

    // Rest of the parameters are set in the setStandardParams function
    setStandardParams();
}

void Metaheuristic::setStandardParams() {
    /*
    Sets the standard parameters for the initilization of the metaheuristic.
    */
    // Set the standard parameters
    d_bestObjVal = std::numeric_limits<double>::max();
    d_sizeNeighborhood = 0;
    d_iRepairOp = 0;
    d_iDestroyOp = 0;
    d_curScore = 0;
    d_parallelIter = 0;
    d_initialize = true;
    d_duals = Duals(d_inst, d_settings);
    d_labels = CapacitatedSortedDeque<Label>(d_settings.d_nColsMaxALNS);
    d_bestSolutions = CapacitatedSortedDeque<ALNSSolution>(d_settings.d_nSolsInit);
    
    // Set the size of the vectors
    d_tempObjVals.reserve(d_settings.d_nALNSIterationsPerNode);
    d_curObjVals.reserve(d_settings.d_nALNSIterationsPerNode);
    d_bestObjVals.reserve(d_settings.d_nALNSIterationsPerNode);
    d_temps.reserve(d_settings.d_nALNSIterationsPerNode);
    d_wDestroyOp.resize(d_settings.d_nDestroyOps);
    d_wRepairOp.resize(d_settings.d_nRepairOps);

    // Set the weights of the destroy and repair operators
    std::fill(d_wDestroyOp.begin(), d_wDestroyOp.end(), 1.0/static_cast<double>(d_settings.d_nDestroyOps));
    std::fill(d_wRepairOp.begin(), d_wRepairOp.end(), 1.0/static_cast<double>(d_settings.d_nRepairOps));

    // Setup SA parameters
    d_temp = d_settings.d_tempStartSA;
}

void Metaheuristic::clear() {
    /*
    Cleanup the metaheuristic after the run.
    */
    d_tempObjVals.clear();
    d_curObjVals.clear();
    d_bestObjVals.clear();
    d_bestObjVal = std::numeric_limits<double>::max();
    d_labels.clear();
    d_parallelIter = 0;
    d_bestSolutions.clear();
    d_bestSol = ALNSSolution(d_inst, d_graph, d_settings);
    d_curSol = ALNSSolution(d_inst, d_graph, d_settings);
    d_tempSol = ALNSSolution(d_inst, d_graph, d_settings);
}

void Metaheuristic::run() {
    /*
    Run the metaheuristic algorithm improving the current solution.
    During an initialization run, the best solutions are stored.
    */
    // Determine the number of iterations based on the mode
    if (d_initialize) {
        d_nIterations = d_settings.d_nALNSInitIterationsPerNode * d_inst.d_nCustNodes;
    }
    else {
        d_nIterations = d_settings.d_nALNSIterationsPerNode * d_inst.d_nCustNodes;
    }
    // Perform nIterations iterations of the metaheuristic
    auto timeStart = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < d_nIterations; i++) {
        setNeighorhoodSize();
        setOperationIdx();
        performDestroy();
        performRepair();
        evaluateSolution(i);
        updateOpWeights();
        if (d_settings.d_verbose) {
            updateProgressBar(i);
        }
    }
    auto timeEnd = std::chrono::high_resolution_clock::now();
    double timeElapsed = std::chrono::duration<double>(timeEnd - timeStart).count();
    if (d_settings.d_verbose) {
        std::cout << "\nBest objective value found: " << d_bestObjVal << " in " << timeElapsed << " seconds\n";
    }
}

void Metaheuristic::setNeighorhoodSize() {
    /*
    Set the size of the neighborhood of the current iteration.
    */
    d_sizeNeighborhood = std::uniform_int_distribution<unsigned short>(1,d_settings.d_maxNeighborhoodSize)(d_gen);
}

void Metaheuristic::findInitialSolution() {
    /*
    Finds the initial solution using n tries of greedy insertion with noise
    */
    bool feasible = false;
    for (unsigned short i = 0; i < d_settings.d_nConstructionTries; i++) {
        // Create a new solution
        d_tempSol = ALNSSolution(d_inst, d_graph, d_settings);
        d_tempSol.sortRequests(d_settings.d_sortType);
        repairGreedy(true);
        if (!d_tempSol.isValid(false, d_duals) || !d_tempSol.isFeasible()) {
            continue;
        }
        if (d_tempSol.d_cost < d_bestObjVal) {
            d_bestSol = d_tempSol;
            d_bestObjVal = d_tempSol.d_cost;
            feasible = true;
            }
        }
    // Store the best solution in the current solution
    d_tempSol = d_bestSol;
    d_curSol = d_bestSol;
    // Store the best solution in the best solutions if in initialize mode
    if (d_initialize) {
        d_bestSolutions.insert(d_bestSol, d_bestSol.hash());
    }
    if (!feasible) {
        std::cout << "\nERROR: No feasible solution found in the construction heuristic. Aborting...\n";
        exit(1);
    }
    if (d_settings.d_verbose) {
        std::cout << "\nConstruction heuristic found solution with objective value "+ std::to_string(d_bestObjVal)+ "\n";
    }

}   

void Metaheuristic::setOperationIdx() {
    /*
    Set the indices of the destroy and repair operators to be used in the next iteration.
    */
    d_iDestroyOp = std::discrete_distribution<unsigned short>(d_wDestroyOp.begin(),d_wDestroyOp.end())(d_gen);
    d_iRepairOp = std::discrete_distribution<unsigned short>(d_wRepairOp.begin(),d_wRepairOp.end())(d_gen);
}

void Metaheuristic::performDestroy() {
    /*
    Performs the destroy operator.
    */
    if (d_sizeNeighborhood >= d_inst.d_nCustNodes - 1) {
        std::cout << "WARNING: Neighbourhood size is larger than or equal to the number of requests. Creating a completely new solution\n";
        d_tempSol = ALNSSolution(d_inst, d_graph, d_settings);
        return;
    }
    unsigned short idAnchor = 0;
    int randomIndex = 0;
    std::unordered_map<unsigned short, VisitLoc>::iterator it;
    double routeCoef = 0.5f;
    switch (d_iDestroyOp) {
        case 0:
            randomIndex = std::uniform_int_distribution<int>(0, static_cast<int>(d_tempSol.d_visitLocs.size() - 1))(d_gen);
            it = std::next(d_tempSol.d_visitLocs.begin(), randomIndex);
            idAnchor = it->first;
            destroyDynamic(idAnchor, routeCoef);
            break;
        case 1:
            destroyGreedy(true);
            break;
        case 2:
            destroyGreedyRoute(true);
            break;
        case 3:
            destroyGreedy(false);
            break;
        case 4:
            destroyRandomRoute();
            break;
        default:
            std::cout << "ERROR: Destroy operator "+std::to_string(d_iDestroyOp)+" not implemented. Aborting...\n";
            exit(1);
    }
}

void Metaheuristic::performRepair() {
    /*
    Performs the repair operator.
    */
    switch (d_iRepairOp) {
        case 0:
            repairGreedy(false);
            break;
        case 1:
            repairGreedy(true);
            break;
        case 2:
            repairRandomFirstFeasible();
            break;
        default:
            std::cout << "ERROR: Repair operator "+std::to_string(d_iRepairOp)+" not implemented. Aborting...\n";
            exit(1);
    }
}

void Metaheuristic::evaluateSolution(unsigned int i) {
    /*
    Evaluate the solution and update the best solution if necessary.
    */
    // Check feasibility and validity of the solution every nCheckFeasible iterations
    bool verified= true;
    bool feasible = true;
    if (i+1 % d_settings.d_nCheckFeasible == 0) {
        verified = d_tempSol.isValid(true, d_duals);
        feasible = d_tempSol.isFeasible();
    }
    // Cases where the solution is not valid or feasible
    d_tempObjVals.push_back(d_tempSol.d_cost);
    if (!verified) {
        std::cout << "WARNING: Solution is in the wrong format. Reverting back to previous solution\n";
        d_curScore = 0;
        d_tempSol = d_curSol;
    }
    else if (!feasible) {
        std::cout << "WARNING: Solution is not feasible at iteration." << i << ". Reverting back to previous solution\n";
        d_curScore = 0;
        d_tempSol = d_curSol;
    }
    // Case where the solution is a global improvement
    else if (d_tempSol.d_cost < d_bestObjVal) {
        d_bestSol = d_tempSol;
        d_curSol = d_tempSol;
        d_bestObjVal = d_tempSol.d_cost;
        d_curScore = 3;
        if (d_initialize) {
            d_bestSolutions.insert(d_tempSol, d_tempSol.hash());
        }
    }
    // Case where the solution is a local improvement
    else if (d_tempSol.d_cost < d_curSol.d_cost) {
        d_curSol = d_tempSol;
        d_curScore = 2;
        if (d_initialize) {
            d_bestSolutions.insert(d_tempSol, d_tempSol.hash());
        }
    }
    // If simulated annealing is used, accept the solution with a certain probability
    else if (d_settings.d_simulatedAnnealing)  {
        double deltaCost = d_tempSol.d_cost - d_curSol.d_cost;
        double p_accept = std::exp(-deltaCost/d_temp);
        double rand = std::uniform_real_distribution<double>(0,1)(d_gen);
        if (rand < p_accept) {
            d_curScore = 1;
            d_curSol = d_tempSol;
            if (d_initialize) {
                d_bestSolutions.insert(d_tempSol, d_tempSol.hash());
            }
        }
        else {
            d_curScore = 0;
            d_tempSol = d_curSol;
        }
    }
    // Case where the solution is not an improvement
    else {
        d_curScore = 0;
        d_tempSol = d_curSol;
    }
    // Add objective value to vector
    d_curObjVals.push_back(d_curSol.d_cost);
    d_bestObjVals.push_back(d_bestSol.d_cost);

    // Update temperature
    if (d_settings.d_simulatedAnnealing) {
        updateSATemp(i);
    }
}

void Metaheuristic::updateOpWeights() {
    /*
    Update the weights of the destroy and repair operators based on curScore.
    The weights remain normalized.
    */
    // Update weights
    if (d_curScore == 0) {
        d_wDestroyOp[d_iDestroyOp] /= (1 + d_settings.d_alnsAlpha);
        d_wRepairOp[d_iRepairOp] /= (1 + d_settings.d_alnsAlpha);

    }
    else {
        d_wDestroyOp[d_iDestroyOp] *= (1 + d_settings.d_alnsAlpha * static_cast<double>(d_curScore));
        d_wRepairOp[d_iRepairOp] *= (1 + d_settings.d_alnsAlpha * static_cast<double>(d_curScore));
    }
    // Normalize weights
    double sumDestroy = std::accumulate(d_wDestroyOp.begin(), d_wDestroyOp.end(), 0.0);
    double sumRepair = std::accumulate(d_wRepairOp.begin(), d_wRepairOp.end(), 0.0);
    for (unsigned short i = 0; i < d_settings.d_nDestroyOps; i++) {
        d_wDestroyOp[i] /= sumDestroy;
    }
    for (unsigned short i = 0; i < d_settings.d_nRepairOps; i++) {
        d_wRepairOp[i] /= sumRepair;
    }
    return;
}

void Metaheuristic::setDuals(Duals duals) {
    /*
    Update the dual variables of the metaheuristic.
    */
    d_duals = duals;
}

void Metaheuristic::storeLabel(unsigned short idDepot, unsigned short idVehicleType, unsigned short idRoute, 
                               unsigned short idNode, unsigned short idSL, unsigned short pos, double reducedCost) {
    /*
    Check the reduced cost of a route and add it to the labels if it is negative.
    */
    // Convert idSL to 255 if it is the maximum value
    if (idSL == std::numeric_limits<unsigned short>::max()) {
        idSL = 255;
    }
    // If it is a new route, create a new label
    if (idRoute == std::numeric_limits<unsigned short>::max()) {
        Label label;
        label.d_idNode = idNode;
        label.d_idVehType = idVehicleType;
        label.d_idDepot = idDepot;
        label.d_visitedNodes = {idDepot, idNode, idDepot};
        label.d_cost = reducedCost;
        label.d_sls.push_back(idSL);
        d_labels.insert(label, label.hash());
        return;
    }
    // Othwerwise, update the existing label
    Label label = d_tempSol.d_depotSols[idDepot].d_vehTypeSols[idVehicleType].d_routes[idRoute].toLabel(d_inst, d_tempSol.d_visitLocs);

    // Insert the new node into the route at the correct position
    label.d_visitedNodes.insert(label.d_visitedNodes.begin() + pos, idNode);
    label.d_cost = reducedCost;

    // SL position is 1-indexed
    pos -= 1;
    label.d_sls.insert(label.d_sls.begin() + pos, idSL);

    // Store the label
    d_labels.insert(label, label.hash());
}

std::deque<Label> Metaheuristic::getLabels() {
    /*
    Return the labels of the negative reduced cost routes.
    */
    if (d_initialize) {
        std::deque<Label> labels;
        std::vector<std::size_t> hashes;
        for (ALNSSolution sol : d_bestSolutions) {
            std::deque<Label> tempLabels = sol.getLabels();
            for (Label label : tempLabels) {
                if (std::find(hashes.begin(), hashes.end(), label.hash()) == hashes.end()) {
                    labels.push_back(label);
                    hashes.push_back(label.hash());
                }
            }
        }
        return labels;
    }
    return d_labels;    
}

void Metaheuristic::updateSATemp(unsigned int i) {
    /*
    Update the temperature of the simulated annealing algorithm.
    */
    std::size_t iParallelIter = i + d_parallelIter * d_nIterations;
    d_temp = d_settings.d_tempEndSA + (d_settings.d_tempStartSA - d_settings.d_tempEndSA) * exp(-10*static_cast<float>(iParallelIter) / static_cast<float>(d_nIterations));
    d_temps.push_back(d_temp);
}

void Metaheuristic::updateProgressBar(unsigned int i) {
    /*
    Update the progress bar in the terminal.
    */
    if (i % 100 == 0) {
    std::cout << "\r" << std::setw(3) << i/((d_settings.d_nALNSIterationsPerNode*d_inst.d_nCustNodes)/100) << "%. Best objective: " << d_bestObjVal;
    std::cout.flush();
    }
}

void Metaheuristic::writeLogs(double timeElapsed, bool error) const {
    /*
    Write the logs of the metaheuristic to a csv file.
    */
    std::cout << "\nWriting logs...\n";
    // Create a folder with the name of the runid
    std::string folder = "output/"+d_idRun;
    std::filesystem::create_directory(folder);
    // Write the solution to a csv file
    writeSolutionTrend(folder+"/objective_trend.csv");
    // Write the summary to a file
    writeSummary(timeElapsed, folder+"/summary.txt");
    // Write the temperatures to a file
    writeTemps(folder+"/temperature_trend.csv");
    // Write the solution to a csv file
    d_bestSol.writeCsv(folder+"/best_solution.csv");
    if (error) {
        d_curSol.writeCsv(folder+"/current_solution.csv");
        d_tempSol.writeCsv(folder+"/temporary_solution.csv");
    }
    return;
}

void Metaheuristic::writeSummary(double timeElapsed, std::string path) const {
    /*
    Write a summary of the results of the metaheuristic to the summary.txt file.
    */
    std::ofstream file;
    unsigned short nRoutes = 0;
    for (unsigned short i = 0; i < d_inst.d_nDepots; i++) {
        for (unsigned short j = 0; j < d_inst.d_nVehTypes; j++) {
            nRoutes += d_bestSol.d_depotSols[i].d_vehTypeSols[j].d_nUsedVehicles;
        }
    }
    double totalDist = 0;
    for (unsigned short i = 0; i < d_inst.d_nDepots; i++) {
        for (unsigned short j = 0; j < d_inst.d_nVehTypes; j++) {
            for (unsigned short k = 0; k < d_bestSol.d_depotSols[i].d_vehTypeSols[j].d_nUsedVehicles; k++) {
                totalDist += d_bestSol.d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_dist;
            }
        }
    }
    file.open(path);
    file << "Instance: " << d_inst.d_name << "\n";
    file << "Best solution found has objective value " << d_bestObjVal << "\n";
    file << "Execution time [s]: " << timeElapsed << "\n";
    file << "Number of routes used: " << nRoutes << "\n";
    file << "Total driven distance: " << totalDist << "\n";
    file << "Destroy weights: \n";
    for (unsigned short i = 0; i < d_wDestroyOp.size(); i++) {
        file << d_wDestroyOp[i] << ", ";
    }
    file << "\n";
    file << "Repair weights: \n";
    for (unsigned short i = 0; i < d_wRepairOp.size(); i++) {
        file << d_wRepairOp[i] << ", ";
    }
    file << "\n";
    file << "Scheduled line usage: \n";
    d_bestSol.writeSLUsage(file);
}

void Metaheuristic::writeSolutionTrend(std::string path) const{
    /*
    Write the solution trend to a csv file.
    */
    std::ofstream file;
    file.open(path);
    file << "iteration,tempObjVal,curObjVal,bestObjVal\n";
    for (unsigned short i = 0; i < d_bestObjVals.size(); i++) {
        file << i << "," << d_tempObjVals[i] << "," << d_curObjVals[i] << "," << d_bestObjVals[i] << "\n";
    }
    file.close();
}

void Metaheuristic::writeTemps(std::string path) const {
    /*
    Write the temperatures of the simulated annealing algorithm to a csv file.
    */
    std::ofstream file;
    file.open(path);
    file << "iteration,temp\n";
    for (unsigned short i = 0; i < d_temps.size(); i++) {
        file << i << "," << d_temps[i] << "\n";
    }
    file.close();
}