#include "metaheuristic.h"

Metaheuristic::Metaheuristic(std::string idRun) {
    /*
    Constructor for the metaheuristic struct.
    */
    // Determine the path of the input and output folder
    std::string pathInput = "", pathOutput = "";
    if (std::filesystem::exists("cpp-metaheuristic")) {
        pathInput = "cpp-metaheuristic/input/";
        pathOutput = "cpp-metaheuristic/output/";
    }
    else if (std::filesystem::exists("input")) {
        pathInput = "input/";
        pathOutput = "output/";
    }
    else {
        std::cout << "ERROR: No input folder found. Please add an input folder to the cpp-metaheuristic directory.\n";
    }
    // Load the settings, instance and graph
    if (d_settings.load(pathInput+"settings.txt"))
        std::cout << "Settings loaded succesfully \n";
    else {
        std::cout << "ERROR: Problem with loading the settings file, aborting... \n";
        exit(1);
    };

    if (d_inst.load(pathInput+"instance.txt")) {
        std::cout << "Instance loaded succesfully \n";
    }
    else {
        std::cout << "ERROR: Problem with loading the settings file, aborting... \n";
        exit(1);
    };

    d_graph.load(d_inst, d_settings);
    d_gen.seed(d_settings.d_randomSeed);
    d_idRun = idRun;
    d_pathOutput = pathOutput;

    // Rest of the parameters are set in the setStandardParams function
    setStandardParams();
}

Metaheuristic::Metaheuristic(Settings &settings, const Instance &inst, const Graph &graph, std::string pathOutput, std::string &idRun, std::size_t seed) {
    /*
    Constructor for the metaheuristic struct.
    */
    d_settings = settings;
    d_idRun = idRun;
    d_pathOutput = pathOutput;
    d_inst = inst;
    d_graph = graph;
    d_gen.seed(seed);

    // Rest of the parameters are set in the setStandardParams function
    setStandardParams();
}

void Metaheuristic::setStandardParams() {
    /*
    Sets the standard parameters for the initilization of the metaheuristic.
    */
    // Set the standard parameters
    d_bestObjVal = std::numeric_limits<float>::max();
    d_sizeNeighborhood = 0;
    d_iRepairOp = 0;
    d_iDestroyOp = 0;
    d_curScore = 0;
    d_nUnimproved = 0;
    d_parallelIter = 0;
    
    // Set the size of the vectors
    d_tempObjVals.reserve(d_settings.d_nIterations*d_settings.d_nThreadLoops);
    d_curObjVals.reserve(d_settings.d_nIterations*d_settings.d_nThreadLoops);
    d_bestObjVals.reserve(d_settings.d_nIterations*d_settings.d_nThreadLoops);
    d_temps.reserve(d_settings.d_nIterations*d_settings.d_nThreadLoops);
    d_wDestroyOp.resize(d_settings.d_nDestroyOps);
    d_wRepairOp.resize(d_settings.d_nRepairOps);

    // Set the weights of the destroy and repair operators
    std::fill(d_wDestroyOp.begin(), d_wDestroyOp.end(), 1.0f/static_cast<float>(d_settings.d_nDestroyOps));
    std::fill(d_wRepairOp.begin(), d_wRepairOp.end(), 1.0f/static_cast<float>(d_settings.d_nRepairOps));

    // Setup SA parameters
    d_temp = d_settings.d_tempStartSA;
}

void Metaheuristic::run() {
    /*
    Run the metaheuristic algorithm improving the current solution.
    */
    // Perform maxIterations iterations of the metaheuristic
    for (std::size_t i = 0; i < d_settings.d_nIterations; i++) {
        setNeighorhoodSize();
        setOperationIdx();
        performDestroy();
        performRepair();
        d_tempSol.patchCosts();
        evaluateSolution(i);
        updateOpWeights();
        if (d_settings.d_verbose) {
            updateProgressBar(i);
        }
    }
}

float Metaheuristic::runSubproblem(std::size_t idSubproblem) {
    /*
    Run the metaheuristic algorithm improving the current solution for a subproblem.
    */
    std::vector<std::size_t> subproblemReqs = getSubproblems()[idSubproblem];
    d_tempSol = d_bestSol;
    d_curSol = d_bestSol;
    // Perform nIteration iterations of the metaheuristic
    for (std::size_t i = 0; i < d_settings.d_nIterations; i++) {
        // Shuffle the subproblem requests
        std::shuffle(subproblemReqs.begin(), subproblemReqs.end(), d_gen);
        // Remove the first nNeighborhood requests from the unassigned requests
        for (std::size_t j = 0; j < d_settings.d_maxNeighborhoodSize; j++) {
            d_tempSol.removeNode(subproblemReqs[j]);
        }
        // Repair the solution
        setOperationIdx();
        performRepair();
        d_tempSol.patchCosts();
        evaluateSolution(i);
        updateOpWeights();
        if (d_settings.d_verbose) {
            updateProgressBar(i);
        }
    }
    return calcSubproblemCost(idSubproblem);
}

float Metaheuristic::calcSubproblemCost(std::size_t idSubproblem) const {
    /*
    Calculate the cost of a subproblem by removing the nodes and finding the delta
    */
    Solution tempSol = d_bestSol;
    std::vector<std::size_t> subproblemReqs = getSubproblems()[idSubproblem];
    float costInit = tempSol.d_cost - tempSol.getPenalty();
    for (std::size_t i = 0; i < subproblemReqs.size(); i++) {
        tempSol.removeNode(subproblemReqs[i]);
    }
    float subproblemCost = costInit - (tempSol.d_cost - tempSol.getPenalty());
    return subproblemCost;
}

void Metaheuristic::setNeighorhoodSize() {
    /*
    Set the size of the neighborhood of the current iteration.
    */
    d_sizeNeighborhood = std::uniform_int_distribution<std::size_t>(1,d_settings.d_maxNeighborhoodSize)(d_gen);
}

void Metaheuristic::findInitialSolution() {
    /*
    Finds the initial solution using n tries of greedy insertion with noise
    */
    bool feasible = false;
    for (std::size_t i = 0; i < d_settings.d_nConstructionTries; i++) {
        // Create a new solution
        d_tempSol = Solution(d_inst, d_graph, d_settings);
        d_tempSol.sortRequests(d_settings.d_sortType);
        if (d_settings.d_restricedServiceArea) {
            repairGreedyLocal(true, d_settings.d_restricedServiceArea);
        }
        else {
            repairGreedy(true);
        }
        if (d_tempSol.isValid(false) && d_tempSol.isFeasible()) {
            feasible = true;
            if (d_tempSol.d_cost < d_bestObjVal) {
                d_bestSol = d_tempSol;
                d_bestObjVal = d_tempSol.d_cost;
            }
        }
    }
    if (!feasible) {
        std::cout << "ERROR: Construction heuristics did not find feasible solution. Stopping...\n";
        exit(1);
    }
    // Store the best solution in the current solution
    d_tempSol = d_bestSol;
    d_curSol = d_bestSol;
    if (d_settings.d_verbose) {
        std::cout << "Construction heuristic found feasible solution with objective value "+ std::to_string(d_bestObjVal)+ "\n";
    }
}   

void Metaheuristic::setOperationIdx() {
    /*
    Set the indices of the destroy and repair operators to be used in the next iteration.
    */
    d_iDestroyOp = std::discrete_distribution<std::size_t>(d_wDestroyOp.begin(),d_wDestroyOp.end())(d_gen);
    d_iRepairOp = std::discrete_distribution<std::size_t>(d_wRepairOp.begin(),d_wRepairOp.end())(d_gen);
}

void Metaheuristic::performDestroy() {
    /*
    Performs the destroy operator.
    */
    if (d_sizeNeighborhood >= d_inst.d_nCustNodes) {
        std::cout << "WARNING: Neighbourhood size is larger than or equal to the number of requests. Creating a completely new solution\n";
        d_tempSol = Solution(d_inst, d_graph, d_settings);
        return;
    }
    std::size_t idAnchor = 0;
    int randomIndex = 0;
    std::unordered_map<std::size_t, VisitLoc>::iterator it;
    float routeCoef = 0.5f;
    switch (d_iDestroyOp) {
        case 0:
            randomIndex = std::uniform_int_distribution<int>(0, static_cast<int>(d_tempSol.d_visitLocs.size() - 1))(d_gen);
            it = std::next(d_tempSol.d_visitLocs.begin(), randomIndex);
            idAnchor = it->first;
            destroyDynamic(idAnchor, routeCoef);
            break;
        case 1:
            destroyRandomRoute();
            break;
        case 2:
            destroyGreedyRoute(true);
            break;
        case 3:
            destroyExcessiveRoute();
            break;
        case 4:
            destroyWaiting();
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
    if (d_settings.d_local) {
        // Perform local repair
        switch (d_iRepairOp) {
            case 0:
                repairGreedyLocal(false, d_settings.d_restricedServiceArea);
                break;
            case 1:
                repairGreedyLocal(true, d_settings.d_restricedServiceArea);
                break;
            case 2:
                repairRandomFirstFeasible(d_settings.d_restricedServiceArea);
                break;
            default:
                std::cout << "ERROR: Repair operator "+std::to_string(d_iRepairOp)+" not implemented. Aborting...\n";
                exit(1);
        }
        return;
    }
    switch (d_iRepairOp) {
        case 0:
            repairGreedy(false);
            break;
        case 1:
            repairGreedy(true);
            break;
        case 2:
            repairRandomFirstFeasible(false);
            break;
        default:
            std::cout << "ERROR: Repair operator "+std::to_string(d_iRepairOp)+" not implemented. Aborting...\n";
            exit(1);
    }
}

void Metaheuristic::evaluateSolution(std::size_t i) {
    /*
    Evaluate the solution and update the best solution if necessary.
    */
    // Check feasibility and validity of the solution every nCheckFeasible iterations
    bool verified= true;
    bool feasible = true;
    if (i % d_settings.d_nCheckFeasible == 0) {
        verified = d_tempSol.isValid(true);
        feasible = d_tempSol.isFeasible();
    }
    // Cases where the solution is not valid or feasible
    d_tempObjVals.push_back(d_tempSol.d_cost);
    if (!verified) {
        std::cout << "WARNING: Solution is in the wrong format. Reverting back to previous solution\n";
        d_curScore = 0;
        d_tempSol = d_curSol;
        d_nUnimproved++;
    }
    else if (!feasible) {
        std::cout << "WARNING: Solution is not feasible at iteration." << i << ". Reverting back to previous solution\n";
        d_curScore = 0;
        d_tempSol = d_curSol;
        d_nUnimproved++;
    }
    // Case where the solution is a global improvement
    else if (d_tempSol.d_cost < d_bestObjVal) {
        d_bestSol = d_tempSol;
        d_curSol = d_tempSol;
        d_bestObjVal = d_tempSol.d_cost;
        d_curScore = 3;
        d_nUnimproved = 0;
    }
    // Case where the solution is a local improvement
    else if (d_tempSol.d_cost < d_curSol.d_cost) {
        d_curSol = d_tempSol;
        d_curScore = 2;
        d_nUnimproved++;
    }
    // If simulated annealing is used, accept the solution with a certain probability
    else if (d_settings.d_simulatedAnnealing)  {
        float deltaCost = d_tempSol.d_cost - d_curSol.d_cost;
        float p_accept = std::exp(-deltaCost/d_temp);
        float rand = std::uniform_real_distribution<float>(0,1)(d_gen);
        if (rand < p_accept) {
            d_curScore = 1;
            d_curSol = d_tempSol;
        }
        else {
            d_curScore = 0;
            d_tempSol = d_curSol;
        }
        d_nUnimproved++;
    }
    // Case where the solution is not an improvement
    else {
        d_curScore = 0;
        d_tempSol = d_curSol;
        d_nUnimproved++;
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
        d_wDestroyOp[d_iDestroyOp] *= (1 + d_settings.d_alnsAlpha * static_cast<float>(d_curScore));
        d_wRepairOp[d_iRepairOp] *= (1 + d_settings.d_alnsAlpha * static_cast<float>(d_curScore));
    }
    // Normalize weights
    float sumDestroy = std::accumulate(d_wDestroyOp.begin(), d_wDestroyOp.end(), 0.0f);
    float sumRepair = std::accumulate(d_wRepairOp.begin(), d_wRepairOp.end(), 0.0f);
    for (std::size_t i = 0; i < d_settings.d_nDestroyOps; i++) {
        d_wDestroyOp[i] /= sumDestroy;
    }
    for (std::size_t i = 0; i < d_settings.d_nRepairOps; i++) {
        d_wRepairOp[i] /= sumRepair;
    }
    return;
}

void Metaheuristic::updateSATemp(std::size_t i) {
    /*
    Update the temperature of the simulated annealing algorithm.
    */
    std::size_t iParallelIter = i + d_parallelIter * d_settings.d_nIterations;
    d_temp = d_settings.d_tempEndSA + (d_settings.d_tempStartSA - d_settings.d_tempEndSA) * exp(-10*static_cast<float>(iParallelIter) / static_cast<float>(d_settings.d_nIterations*d_settings.d_nThreadLoops));
    d_temps.push_back(d_temp);
}

void Metaheuristic::updateProgressBar(std::size_t i) {
    /*
    Update the progress bar in the terminal.
    */
    if (i % (d_settings.d_nIterations/100) == 0) {
    std::cout << "\r" << std::setw(3) << i/(d_settings.d_nIterations/100) << "%. Best objective: " << d_bestObjVal;
    std::cout.flush();
    }
}

const std::vector<std::vector<std::size_t>>& Metaheuristic::getSubproblems() const {
    /*
    Get the subproblems of the instance.
    */
    return d_bestSol.d_subproblems;
}

std::size_t Metaheuristic::getSubproblemCount() const {
    /*
    Returns the number of subproblems
    */
    return d_bestSol.d_subproblems.size();
}

Solution Metaheuristic::getBestSolution() const {
    /*
    Get the best solution of the metaheuristic.
    */
    return d_bestSol;
}

void Metaheuristic::setBestSolution(Solution sol) {
    /*
    Set the best solution of the metaheuristic.
    */
    d_bestSol = sol;
    d_bestObjVal = sol.d_cost;
}

void Metaheuristic::updateSubproblems() {
    /*
    Update the subproblems of the instance.
    */
    d_bestSol.updateSubproblems();
}

std::vector<std::vector<float>> Metaheuristic::getSubproblemEmbeddings(std::size_t idSubproblem) const {
    /* 
    Given the id of the subproblem, return the embedding
    */
    // index the instance d_embeddings list using the subproblem ids to obtain the relevant embeddings
    std::vector<std::vector<float>> embeddings(d_bestSol.d_subproblems[idSubproblem].size());
    for (std::size_t i = 0; i < d_bestSol.d_subproblems[idSubproblem].size(); i++) {
        embeddings[i] = d_inst.d_nodeEmbeddings[d_bestSol.d_subproblems[idSubproblem][i]];
    }
    return embeddings;
}

void Metaheuristic::writeLogs(float timeElapsed, bool error, bool python) const {
    /*
    Write the logs of the metaheuristic to a csv file.
    */
    std::cout << "\nWriting logs...\n";
    // Create a folder with the name of the runid
    std::string folder = "output/"+d_idRun;
    if (python) {
        folder = "cpp-metaheuristic/output/"+d_idRun;
    }
    std::filesystem::create_directory(folder);
    // Write the solution to a csv file
    writeSolutionTrend(folder+"/log"+d_settings.d_instName+".csv");
    // Write the summary to a file
    writeSummary(timeElapsed, folder+"/summary.txt");
    // Write the temperatures to a file
    writeTemps(folder+"/temps"+d_settings.d_instName+".csv");
    // Write the solution to a csv file
    d_bestSol.writeCsv(folder+"/sol"+d_settings.d_instName+".csv");
    if (error) {
        d_curSol.writeCsv(folder+"/curSol"+d_settings.d_instName+".csv");
        d_tempSol.writeCsv(folder+"/tempSol"+d_settings.d_instName+".csv");
    }
    //Write the solution to a JSON file
    d_bestSol.writeJSON(folder+"/sol_"+d_settings.d_instName+".json");
}

void Metaheuristic::writeSummary(float timeElapsed, std::string path) const {
    /*
    Write a summary of the results of the metaheuristic to the summary.txt file.
    */
    std::ofstream file;
    std::size_t nRoutes = 0;
    for (std::size_t i = 0; i < d_inst.d_nDepots; i++) {
        for (std::size_t j = 0; j < d_inst.d_nVehTypes; j++) {
            nRoutes += d_bestSol.d_depotSols[i].d_vehTypeSols[j].d_nUsedVehicles;
        }
    }
    float totalDist = 0;
    for (std::size_t i = 0; i < d_inst.d_nDepots; i++) {
        for (std::size_t j = 0; j < d_inst.d_nVehTypes; j++) {
            for (std::size_t k = 0; k < d_bestSol.d_depotSols[i].d_vehTypeSols[j].d_nUsedVehicles; k++) {
                totalDist += d_bestSol.d_depotSols[i].d_vehTypeSols[j].d_routes[k].d_dist;
            }
        }
    }
    std::size_t qSL = 0;
    std::size_t capSL = 0;
    for (std::size_t i = 0; i < d_inst.d_nSL; i++) {
        qSL += d_bestSol.d_SLSols[i].d_qAssigned;
        capSL += d_inst.d_scheduledLines[i].d_capacity;
    }
    float uSL = 100.0f*static_cast<float>(qSL)/static_cast<float>(capSL);
    file.open(path);
    file << "Instance: " << d_settings.d_instName << "\n";
    file << "Run ID: " << d_idRun << "\n";
    file << "Best solution found has objective value " << d_bestObjVal << "\n";
    file << "Execution time [s]: " << timeElapsed << "\n";
    file << "Number of routes used: " << nRoutes << "\n";
    file << "Total driven distance: " << totalDist << "\n";
    file << "Average utilization of scheduled lines: " << uSL << "\n";
    file << "Destroy weights: \n";
    for (std::size_t i = 0; i < d_wDestroyOp.size(); i++) {
        file << d_wDestroyOp[i] << ", ";
    }
    file << "\n";
    file << "Repair weights: \n";
    for (std::size_t i = 0; i < d_wRepairOp.size(); i++) {
        file << d_wRepairOp[i] << ", ";
    }
    file << "\n";
    file << "Scheduled line usage: \n";
    d_bestSol.writeSLUsage(file);
    if (d_settings.d_restricedServiceArea) {
        std::size_t nInf = d_bestSol.getNAreaInfeasible();
        file << "Number of infeasible requests in restricted service area: " << nInf << "\n";
        file << "Infeasible requests: ";
        for (std::size_t i = 0; i < d_bestSol.d_reqInfeasible.size(); i++) {
            file << d_bestSol.d_reqInfeasible[i] << ", ";
        }
    }
}

void Metaheuristic::writeSolutionTrend(std::string path) const{
    /*
    Write the solution trend to a csv file.
    */
    std::ofstream file;
    file.open(path);
    file << "iteration,tempObjVal,curObjVal,bestObjVal\n";
    for (std::size_t i = 0; i < d_bestObjVals.size(); i++) {
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
    for (std::size_t i = 0; i < d_temps.size(); i++) {
        file << i << "," << d_temps[i] << "\n";
    }
    file.close();
}