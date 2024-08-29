#include "mpGurobi.hpp"

MasterProblemGRB::MasterProblemGRB(Settings& settings, const Instance& instance, const Graph& graph) :
d_settings(settings), d_inst(instance), d_graph(graph)
{
    if (settings.d_verbose)
        d_env.set(GRB_IntParam_OutputFlag, 1);
    else
        d_env.set(GRB_IntParam_OutputFlag, 0);
    d_env.set(GRB_IntParam_Method, 0);        // 0 for primal simplex, 1 for dual simplex
    d_env.set(GRB_IntParam_Cuts, 0);          // Disable cuts
    d_env.set(GRB_IntParam_Presolve, 0);      // Disable presolve
    d_env.set(GRB_IntParam_BarHomogeneous, 0);// Disable barrier method
    d_env.set(GRB_IntParam_Crossover, 0);     // Disable crossover to Simplex from barrier method
    d_env.set(GRB_DoubleParam_Heuristics, 0);
    d_solRelaxed = LPSolution(d_inst, d_settings);
    try        
    {
        d_nVariables = 0;
        d_nCustConstraints = 0;
        d_nVehConstraints = 0;
        d_nSLConstraints = 0;

        d_env.start();
    }
    catch (GRBException& e)
    {
        std::cerr << "Gurobi Error occurred during MasterProblem initialization: " << e.getMessage() << std::endl;
        std::cerr << "Error code: " << e.getErrorCode() << std::endl;
        throw;
    }
    catch (std::exception& e)
    {
        std::cerr << "Standard exception occurred during MasterProblem initialization: " << e.what() << std::endl;
        throw;
    }
}

void MasterProblemGRB::clear()
{
    /*
    Clear the current MP
    */
    d_xVariables.clear();
    d_routes.clear();
    d_routeVehTypes.clear();
    d_routesInMP.clear();
    d_nVariables = 0;
    d_nCustConstraints = 0;
    d_nVehConstraints = 0;
    d_nSLConstraints = 0;
    d_solRelaxed = LPSolution(d_inst, d_settings);
}

void MasterProblemGRB::initDummyVar()
{
    Variable var = Variable();

    std::vector<double> coefs;

    for (unsigned short cust_idx = 0; cust_idx < d_inst.d_nCustNodes; cust_idx++)
    {
        coefs.emplace_back(1.0);
    }

    for (unsigned short idx = 0; idx < d_inst.d_vehTypesInfo.size(); idx++)
    {
        coefs.emplace_back(0.0);
    }

    for (unsigned short sl_idx = 0; sl_idx < d_inst.d_nSL; sl_idx++)
    {
        coefs.emplace_back(0.0);
    }

    var.d_coefObjective = d_settings.d_bigM;
    var.d_coefsConstaint = coefs;

    d_xVariables.push_back(var);
    d_nVariables++;

    return;
}

void MasterProblemGRB::initALNS() {
    Metaheuristic meta = Metaheuristic(d_settings, d_inst, d_graph, d_settings.d_randomSeed);
    // Initialize using ALNS
    meta.d_initialize = true;
    meta.setDuals(d_solRelaxed.d_duals);
    meta.findInitialSolution();
    meta.run();
    std::deque<Label> labels = meta.getLabels();
    addPricedVariables(labels);
}

void MasterProblemGRB::labelExact(Result& result, std::atomic<bool>& stopFlag)
{
    /*
    Labeling algorithm for the exact method
    */

    std::vector<std::pair<unsigned short, unsigned short>> subproblems;

    for (unsigned short idDepot = 0; idDepot < d_inst.d_nDepots; idDepot++)
    {
        for (unsigned short idVehType = 0; idVehType < d_inst.d_nVehTypes; idVehType++)
        {
            subproblems.emplace_back(std::pair<unsigned short, unsigned short>(idDepot, idVehType));
        }
    }
    unsigned short ctrNotFoundColumns = 0;
    unsigned short idSubproblem = 0;
    unsigned short ctrPricingExact = 0;

    while(ctrNotFoundColumns < d_inst.d_nDepots * d_inst.d_nVehTypes)
    {
        solveCurrentLP(true);

        unsigned short idDepot = subproblems[idSubproblem].first;
        unsigned short idVehType = subproblems[idSubproblem].second;
        Pricer pricer = Pricer(d_solRelaxed, d_settings, d_inst, d_graph);

        pricer.setSubProblem(idDepot, idVehType);
        pricer.solveExact(stopFlag, d_settings.d_nColsMaxExact);

        if (stopFlag.load())
        {
            return;
        }

        std::deque<Label> labels = pricer.getCompleteLabels();

        bool foundNewCols = addPricedVariables(labels);

        if (foundNewCols)
        {
            ctrNotFoundColumns = 0;
            ctrPricingExact++;
        }
        else
        {
            ctrNotFoundColumns++;
        }
        
        idSubproblem++;

        if (idSubproblem == subproblems.size())
        {
            idSubproblem = 0;
        }

        if (d_settings.d_verbose)
        {
            std::cout << "Exact pricing loop: " << ctrPricingExact <<  "." << std::endl;
        }
    }

    // Store LB results
    result.d_lbExact = d_solRelaxed.d_objective;
    result.d_nPricingExact = ctrPricingExact;
    result.d_exactInteger = d_solRelaxed.isInteger();

    // If the solution is not integer, solve the integer solution
    if (!d_solRelaxed.isInteger())
    {
        solveCurrentLP(false);
        result.d_ubMPExact = d_solInteger.d_objective;
    }
    else
    {
        d_solInteger = d_solRelaxed;
        result.d_ubMPExact = result.d_lbExact;
    }    
}

void MasterProblemGRB::labelALNS(Result& result, std::atomic<bool>& stopFlag)
{
    Metaheuristic meta = Metaheuristic(d_settings, d_inst, d_graph, d_settings.d_randomSeed);

    // Start the labeling algorithm
    bool verifiedExact = false;
    meta.d_initialize = false;
    unsigned short ctrALNS = 0;
    unsigned short ctrExact = 0;

    // Store value, to reset after ALNS
    unsigned short nIterationsPerNode = d_settings.d_nALNSIterationsPerNode;

    while (!verifiedExact) 
    {
        // Start adding columns using ALNS until no new columns are found
        unsigned short ctrNotFoundColumns = 0;
        while (ctrNotFoundColumns <= d_settings.d_nRestartALNS)
        {
            solveCurrentLP(true);
            meta.setDuals(d_solRelaxed.d_duals);
            meta.findInitialSolution();
            meta.run();
            bool foundNewColumns = addPricedVariables(meta.getLabels());
            if (!foundNewColumns) 
            {
                // No new columns found, increase number of iterations and restart
                d_settings.d_nALNSIterationsPerNode *= 2;
                ctrNotFoundColumns++;
            }
            else {
                ctrNotFoundColumns = 0;
            }
            meta.clear();
            ctrALNS++;
        }
        d_settings.d_nALNSIterationsPerNode = nIterationsPerNode;
        if (d_settings.d_verifyExact) 
        {
            unsigned short ctrSubproblems = 0;
            bool foundNewColumns = false;
            for (unsigned short i = 0; i < d_inst.d_nDepots; i++)
            {
                if (foundNewColumns)
                {
                    break;
                }
                for (unsigned short j = 0; j < d_inst.d_nVehTypes; j++) 
                {
                    Pricer pricer = Pricer(d_solRelaxed, d_settings, d_inst, d_graph);
                    pricer.setSubProblem(i,j);
                    pricer.solveExact(stopFlag, d_settings.d_nColsMaxExact);
                    if (stopFlag.load())
                    {
                        return;
                    }
                    if (addPricedVariables(pricer.getCompleteLabels()))
                    {
                        ctrExact++;
                        foundNewColumns = true;
                        break;
                    }
                    else {
                        ctrSubproblems++;
                    }
                }
            }
            if (d_settings.d_verbose)
            {
                auto timeEnd = std::chrono::high_resolution_clock::now();
                std::cout << "Exact pricing loops: " << ctrExact <<  "." << std::endl;
            }
            if (ctrSubproblems == static_cast<unsigned short>(d_inst.d_nDepots * d_inst.d_nVehTypes)) 
            {
                verifiedExact = true;
            }
            else
            {
                ctrSubproblems = 0;
                meta.clear();
            }
        }
        else {
            verifiedExact = true;
        }
    }
    // Store LB results
    result.d_lbALNS = d_solRelaxed.d_objective;
    result.d_nPricingALNS = ctrALNS;
    result.d_nPricingALNSExact = ctrExact;
    result.d_alnsInteger = d_solRelaxed.isInteger();
    result.d_nCustPerRoutePricing = countCustomersPerRoute();

    // If the solution is not integer, solve the integer solution
    if (!d_solRelaxed.isInteger())
    {
        solveCurrentLP(false);
        result.d_ubMPALNS = d_solInteger.d_objective;
    }
    else
    {
        d_solInteger = d_solRelaxed;
        result.d_ubMPALNS = result.d_lbALNS;
    }

    // If we want to write files, write the solution
    if (d_settings.d_writeSols)
    {
        writeSolution();
    }
}

double MasterProblemGRB::countCustomersPerRoute()
{
    unsigned short nCust = 0;
    unsigned short nRoutes = 0;

    for (unsigned short idVar = 1; idVar < d_solRelaxed.d_variableValues.size(); idVar++)
    {
        if (d_solRelaxed.d_variableValues[idVar] > d_settings.d_tolerance)
        {
            nRoutes++;
            const Label& route = d_routes[idVar - 1];
            for (unsigned short idNode = 1; idNode < route.d_visitedNodes.size() - 1; idNode++)
            {
                nCust++;
            }
        }
    }

    double nCustPerRoute = nCust / nRoutes;

    return nCustPerRoute;
}

void MasterProblemGRB::validateReducedCostOfVariables()
{
    /*
    Validate if reduced cost of current variables corresponds with the duals Gurobi gives
    */
    for (unsigned short route_idx = 0; route_idx < d_routes.size(); route_idx++)
    {
        const Label& route = d_routes[route_idx];

        unsigned short idDept = route.d_visitedNodes[0];
        unsigned short idVehType = d_routeVehTypes[route_idx];

        double costRealDist = 0.0;
        double costRealSL = 0.0;

        double reducedCostCust = 0.0;
        double reducedCostVeh = -d_solRelaxed.d_duals.d_vehicle[idDept * d_inst.d_nVehTypes + idVehType];
        double reducedCostSL = 0.0;

        for (unsigned short idVisit = 1; idVisit < route.d_visitedNodes.size(); idVisit++)
        {
            unsigned short idNodeFrom = route.d_visitedNodes[idVisit - 1];
            unsigned short idNodeTo = route.d_visitedNodes[idVisit];

            costRealDist += d_graph.getDist(idVehType, idNodeFrom, idNodeTo) * d_inst.d_vehTypes[idVehType].d_costVariable;

            if (idNodeTo < d_inst.d_nDepots)
            {
                break;
            }

            unsigned short idCust = idNodeTo - d_inst.d_nDepots;

            reducedCostCust -= d_solRelaxed.d_duals.d_customer[idCust];

            unsigned char idSL = route.d_sls[idVisit - 1];

            if (d_inst.d_nodes[idNodeTo].d_idDepot != idDept)
            {
                assert(idSL != 255);

                double qty = d_inst.d_nodes[idNodeTo].d_qCol + d_inst.d_nodes[idNodeTo].d_qDel;

                costRealSL += qty * d_inst.d_scheduledLines[idSL].d_costRequest;
                reducedCostSL -= qty * d_solRelaxed.d_duals.d_sl[idSL];
            }
        }

        double reducedCostTot = (
            costRealDist +
            costRealSL +
            reducedCostCust +
            reducedCostVeh +
            reducedCostSL + 
            d_inst.d_vehTypes[route.d_idVehType].d_costFixed
        );

        double reducedCostVar = d_solRelaxed.d_variableReducedCosts[route_idx + 1];

        if (!(reducedCostTot > reducedCostVar - d_settings.d_tolerance && reducedCostTot < reducedCostVar + d_settings.d_tolerance))
        {
            throw std::runtime_error("Reduced cost calculation after LP incorrect.");
        }
    }

    return;
}

void MasterProblemGRB::validateReducedCost(const std::deque<Label>& paths)
{
    /*
    Recalculate reduced cost and provide error if incorrect.

    :param paths:   paths to check the reduced cost for
    :param sol:     solution with duals
    */
    for (const Label route : paths)
    {
        // recalc
        double realCostDist = 0.0;
        double realCostSL = 0.0;

        double reducedCostCust = 0.0;
        double reducedCostVeh = -d_solRelaxed.d_duals.d_vehicle[route.d_idDepot * d_inst.d_nVehTypes + route.d_idVehType];
        double reducedCostSL = 0.0;

        for (unsigned short idVisit = 1; idVisit < route.d_visitedNodes.size(); idVisit++)
        {
            unsigned short idNodeFrom = route.d_visitedNodes[idVisit - 1];
            unsigned short idNodeTo = route.d_visitedNodes[idVisit];

            realCostDist += d_graph.getDist(route.d_idVehType, idNodeFrom, idNodeTo) * d_inst.d_vehTypes[route.d_idVehType].d_costVariable;

            if (idNodeTo < d_inst.d_nDepots)
            {
                break;
            }

            unsigned short idCust = idNodeTo - d_inst.d_nDepots;

            reducedCostCust -= d_solRelaxed.d_duals.d_customer[idCust];

            unsigned char idSL = route.d_sls[idVisit - 1];

            if (d_inst.d_nodes[idNodeTo].d_idDepot != route.d_idDepot)
            {
                assert(idSL != 255);

                double qty = d_inst.d_nodes[idNodeTo].d_qCol + d_inst.d_nodes[idNodeTo].d_qDel;

                realCostSL += qty * d_inst.d_scheduledLines[idSL].d_costRequest;
                reducedCostSL -= qty * d_solRelaxed.d_duals.d_sl[idSL];
            }
        }

        double reducedCostTotal = (
            realCostDist +
            realCostSL +
            reducedCostCust +
            reducedCostVeh +
            reducedCostSL + 
            d_inst.d_vehTypes[route.d_idVehType].d_costFixed
        );

        if (!(reducedCostTotal > route.d_cost - d_settings.d_tolerance && reducedCostTotal < route.d_cost + d_settings.d_tolerance))
        {
            throw std::runtime_error("Reduced cost calculation incorrect.");
        }
    }

    return;
}

void MasterProblemGRB::printSolution()
{
    /*
    Print routes in the current solution
    */
    if (d_solRelaxed.d_variableValues[0] > d_settings.d_tolerance)
    {
        std::cout << "No feasible solution found! Dummy variable selected with " << d_solRelaxed.d_variableValues[0] << std::endl;
    }

    for (unsigned short idVar = 1; idVar < d_nVariables; idVar++)
    {
        if (d_solRelaxed.d_variableValues[idVar] < d_settings.d_tolerance)
        {
            continue;
        }

        const Label& label = d_routes[idVar - 1];

        std::cout << idVar << ", " << d_solRelaxed.d_variableValues[idVar] << " nodes: {";

        for (unsigned short idVisit = 0; idVisit < label.d_visitedNodes.size(); idVisit++)
        {
            std::cout << label.d_visitedNodes[idVisit];

            if (idVisit < label.d_visitedNodes.size() - 1)
            {
                std::cout << ", ";
            }
        }

        std::cout << "}" << std::endl;
    }

    return;
}

void MasterProblemGRB::writeSolution() {
    /*
    Write the solution to a file
    */
    // Create a solution directory if it does not exist
    std::filesystem::create_directory("output/solutions");
    // Write the solution to a file
    std::ofstream file("output/solutions/sol_" + d_inst.d_name + ".json");
    if (file.is_open()) {
        file << "{\n";
        file << "  \"Instance\": \"" << d_inst.d_name << "\",\n";
        file << "  \"Algorithm\": \"";
        if (d_settings.d_verifyExact) {
            file << "Exact"<< "\",\n";
        }
        else {
            file << "Heuristic" << "\",\n";
        }
        file << "  \"Relaxed Master Problem Solution\": {\n";
        file << "    \"Objective\": " << d_solRelaxed.d_objective << ",\n";
        file << "    \"Variables\": [\n";
        int ctrVars = 0;
        for (unsigned short idVar = 1; idVar < d_nVariables; idVar++) {
            if (d_solRelaxed.d_variableValues[idVar] < d_settings.d_tolerance) {
                continue;
            }
            if (ctrVars > 0) {
                file << ",\n";
            }
            ctrVars++;
            const Label& label = d_routes[idVar - 1];
            file << "      {\n";
            file << "        \"weight\": " << d_solRelaxed.d_variableValues[idVar] << ",\n";
            file << "        \"nodes\": [";
            for (unsigned short idVisit = 0; idVisit < label.d_visitedNodes.size(); idVisit++) {
                file << label.d_visitedNodes[idVisit];
                if (idVisit < label.d_visitedNodes.size() - 1) {
                    file << ", ";
                }
            }
            file << "],\n";
            file << "        \"scheduled lines\": [";
            for (unsigned short idVisit = 0; idVisit < label.d_sls.size(); idVisit++) {
                if (label.d_sls[idVisit] == 255) {
                    file << "-1";
                }
                else {
                    file << static_cast<int>(label.d_sls[idVisit]);
                }
                if (idVisit < label.d_sls.size() - 1) {
                    file << ", ";
                }
            }
            file << "]\n";
            file << "      }";
        }
        file << "    ]\n";
        file << "  },\n";
        file << "  \"Integer Master Problem Solution\": {\n";
        file << "    \"Objective\": " << d_solInteger.d_objective << ",\n";
        file << "    \"Variables\": [\n";
        ctrVars = 0;
        for (unsigned short idVar = 1; idVar < d_nVariables; idVar++) {
            if (d_solInteger.d_variableValues[idVar] < d_settings.d_tolerance) {
                continue;
            }
            if (ctrVars > 0) {
                file << ",\n";
            }
            ctrVars++;
            const Label& label = d_routes[idVar - 1];
            file << "      {\n";
            file << "        \"weight\": " << d_solInteger.d_variableValues[idVar] << ",\n";
            file << "        \"nodes\": [";
            for (unsigned short idVisit = 0; idVisit < label.d_visitedNodes.size(); idVisit++) {
                file << label.d_visitedNodes[idVisit];
                if (idVisit < label.d_visitedNodes.size() - 1) {
                    file << ", ";
                }
            }
            file << "],\n";
            file << "        \"scheduled lines\": [";
            for (unsigned short idVisit = 0; idVisit < label.d_sls.size(); idVisit++) {
                if (label.d_sls[idVisit] == 255) {
                    file << "-1";
                }
                else {
                    file << static_cast<int>(label.d_sls[idVisit]);
                }
                if (idVisit < label.d_sls.size() - 1) {
                    file << ", ";
                }
            }
            file << "]\n";
            file << "      }";
        }
        file << "    ]\n";
        file << "  }\n";

        file << "}";
        file.close();
    }
    else {
        std::cerr << "Unable to open file for writing." << std::endl;
    }
}
bool MasterProblemGRB::addPricedVariables(const std::deque<Label>& paths)
{
    /*
    Add priced variables to the problem
    */
    int ctrRoutes = 0;

    if (paths.empty())
    {
        return false;
    }

    for (const Label &route : paths)
    {
        auto result = d_routesInMP.insert(std::tuple<unsigned short, std::vector<unsigned short>, std::vector<unsigned char>>(
            route.d_idVehType,
            route.d_visitedNodes,
            route.d_sls
        ));
        if (!result.second)
        {
            throw std::runtime_error("Route already present in MIP");
        }
        ctrRoutes++;
        double costVehType = d_inst.d_vehTypes[route.d_idVehType].d_costVariable; // Variable cost per meter

        double realCost = d_inst.d_vehTypes[route.d_idVehType].d_costFixed;
        double costSL = 0.0;
        double routingCost = 0.0;

        std::vector<double> coefConstraintCustomer(d_inst.d_nCustNodes, 0.0);
        std::vector<double> coefConstainsVehicle(d_inst.d_vehTypesInfo.size(), 0.0);
        std::vector<double> coefConstraintSL(d_inst.d_nSL, 0.0);

        coefConstainsVehicle[route.d_idDepot * d_inst.d_nVehTypes + route.d_idVehType] = 1.0;

        for (unsigned short idRoute = 0; idRoute < route.d_visitedNodes.size() - 1; idRoute++)
        {
            unsigned short nodeFrom = route.d_visitedNodes[idRoute];
            unsigned short nodeTo = route.d_visitedNodes[idRoute + 1];

            realCost += d_graph.getDist(route.d_idVehType, nodeFrom, nodeTo) * costVehType;
            routingCost += d_graph.getDist(route.d_idVehType, nodeFrom, nodeTo) * costVehType;

            if (static_cast<unsigned short>(idRoute + 1) == route.d_visitedNodes.size() - 1)
            {
                break; // back to depot
            }

            assert(nodeTo - d_inst.d_nDepots >= 0);
            unsigned short idCust = nodeTo - d_inst.d_nDepots;

            coefConstraintCustomer[idCust] = 1.0;

            if (d_inst.d_nodes[nodeTo].d_idDepot != route.d_idDepot)
            {
                unsigned short qty = d_inst.d_nodes[nodeTo].d_qCol + d_inst.d_nodes[nodeTo].d_qDel;
                unsigned char idSL = route.d_sls[idRoute];

                assert(idSL != 255);

                double sl_cost = qty * d_inst.d_scheduledLines[idSL].d_costRequest;

                realCost += sl_cost;
                costSL += sl_cost;

                coefConstraintSL[idSL] = qty;
            }
        }

        #ifndef N_DEBUG
        validateRealCost(realCost, route);
        #endif

        std::vector<double> coefs;

        coefs.insert(coefs.end(), coefConstraintCustomer.begin(), coefConstraintCustomer.end());
        coefs.insert(coefs.end(), coefConstainsVehicle.begin(), coefConstainsVehicle.end());
        coefs.insert(coefs.end(), coefConstraintSL.begin(), coefConstraintSL.end());

        Variable var = Variable();

        var.d_coefObjective = realCost;
        var.d_coefsConstaint = coefs;

        d_xVariables.push_back(var);
        d_routes.push_back(route);
        d_routeVehTypes.push_back(route.d_idVehType);

        d_nVariables++;
    }
    
    return  true;
}

void MasterProblemGRB::validateRealCost(double costReal, const Label &route)
{
    // recalc
    double costRealDist = 0.0;
    double costRealSL = 0.0;

    for (unsigned short idVisit = 1; idVisit < route.d_visitedNodes.size(); idVisit++)
    {
        unsigned short idNodeFrom = route.d_visitedNodes[idVisit - 1];
        unsigned short idNodeTo = route.d_visitedNodes[idVisit];

        costRealDist += d_graph.getDist(route.d_idVehType, idNodeFrom, idNodeTo) * d_inst.d_vehTypes[route.d_idVehType].d_costVariable;

        if (idNodeTo < d_inst.d_nDepots)
        {
            break;
        }

        unsigned char idSL = route.d_sls[idVisit - 1];

        if (d_inst.d_nodes[idNodeTo].d_idDepot != route.d_idDepot)
        {
            assert(idSL != 255);

            double qty = d_inst.d_nodes[idNodeTo].d_qCol + d_inst.d_nodes[idNodeTo].d_qDel;

            costRealSL += qty * d_inst.d_scheduledLines[idSL].d_costRequest;
        }
    }

    double totalCost = (
        costRealDist +
        costRealSL +
        d_inst.d_vehTypes[route.d_idVehType].d_costFixed
    );

    if (!(totalCost > costReal - d_settings.d_tolerance && totalCost < costReal + d_settings.d_tolerance))
    {
        throw std::runtime_error("Real cost calculation incorrect.");
    }

    return;
}

void MasterProblemGRB::solveCurrentLP(bool relaxed)
{
    /*
    Solve current MP
    */
    // In the case of an integer solve, do generate logs
    GRBModel model(d_env);

    std::vector<GRBVar> grbVars;
    std::vector<GRBConstr> grbConstrs;

    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

    for (unsigned short idVar = 0; idVar < d_nVariables; idVar++)
    {
        const Variable& var = d_xVariables[idVar];

        if (relaxed)
        {
            grbVars.emplace_back(
                model.addVar(0.0, GRB_INFINITY, var.d_coefObjective, GRB_CONTINUOUS)
            );
        }
        else
        {
            grbVars.emplace_back(
                model.addVar(0.0, GRB_INFINITY, var.d_coefObjective, GRB_INTEGER)
            );
        }
    }

    int idConstrOverall = 0;

    for (unsigned short idCust = 0; idCust < d_inst.d_nCustNodes; idCust++)
    {
        GRBLinExpr expr = 0;

        for (unsigned short idVar = 0; idVar < d_nVariables; idVar++)
        {
            expr += d_xVariables[idVar].d_coefsConstaint[idConstrOverall] * grbVars[idVar];
        }

        grbConstrs.emplace_back(
            model.addConstr(expr >= 1)
        );
        
        idConstrOverall++;
    }

    for (unsigned short idx = 0; idx < d_inst.d_vehTypesInfo.size(); idx++)
    {
        GRBLinExpr expr = 0;

        for (unsigned short idVar = 0; idVar < d_nVariables; idVar++)
        {
            expr += d_xVariables[idVar].d_coefsConstaint[idConstrOverall] * grbVars[idVar];
        }

        grbConstrs.emplace_back(
            model.addConstr(expr <= d_inst.d_vehTypesInfo[idx].d_nVehicles)
        );

        idConstrOverall++;
    }

    for (unsigned short idSL = 0; idSL < d_inst.d_nSL; idSL++)
    {
        GRBLinExpr expr = 0;

        for (unsigned short idVar = 0; idVar < d_nVariables; idVar++)
        {
            expr += d_xVariables[idVar].d_coefsConstaint[idConstrOverall] * grbVars[idVar];
        }

        grbConstrs.emplace_back(
            model.addConstr(expr <= d_inst.d_scheduledLines[idSL].d_capacity)
        );

        idConstrOverall++;
    }

    model.update();
    model.optimize();
    if (model.get(GRB_IntAttr_Status) == GRB_INFEASIBLE)
    {
    // Compute the Irreducible Inconsistent Subsystem (IIS)
        model.computeIIS();
        model.write("model.ilp");
        throw std::runtime_error("Model is infeasible. See listed constraints and variable bounds in model.ilp");
    } 
    else if (model.get(GRB_IntAttr_Status) != GRB_OPTIMAL) 
    {
        throw std::runtime_error("Master Problem Gurobi error: Model not optimal.");
    }

    LPSolution sol(d_inst, d_settings);

    sol.d_objective = model.get(GRB_DoubleAttr_ObjVal);

    for (unsigned short idVar = 0; idVar < d_nVariables; idVar++)
    {
        sol.d_variableValues.push_back(
            grbVars[idVar].get(GRB_DoubleAttr_X)
        );

        if (relaxed)
        // No reduced costs for integer solutions
        {
            sol.d_variableReducedCosts.push_back(
                grbVars[idVar].get(GRB_DoubleAttr_RC)
            );
        }
    }

    if (!relaxed) {
        // No duals for integer solutions, return
        d_solInteger = sol;
        return;
    }

    int idConstr = 0;

    for (unsigned short idCust = 0; idCust < d_inst.d_nCustNodes; idCust++)
    {
        double dual = grbConstrs[idConstr].get(GRB_DoubleAttr_Pi);

        sol.d_duals.d_customer[idCust] = dual;

        idConstr++;
    }

    for (unsigned short idx = 0; idx < d_inst.d_vehTypesInfo.size(); idx++)
    {
        double dual = grbConstrs[idConstr].get(GRB_DoubleAttr_Pi);

        sol.d_duals.d_vehicle[idx] = dual;

        idConstr++;
    }

    for (unsigned short idSL = 0; idSL < d_inst.d_nSL; idSL++)
    {
        double dual = grbConstrs[idConstr].get(GRB_DoubleAttr_Pi);

        sol.d_duals.d_sl[idSL] = dual;

        idConstr++;
    }

    d_solRelaxed = sol;
    // Recalc reduced cost of current variables
    #ifndef N_DEBUG
    validateReducedCostOfVariables();
    #endif
}