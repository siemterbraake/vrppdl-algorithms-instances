#include "metaheuristic.h"

void Metaheuristic::patchInfeasibleRequests() {
    /*
    Algorithm for insertion feasible requests, removing infeasible requests and inserting them again.
    This function is used in the case of strict service area enforcement.
    */
    // Store the list of infeasible requests
    std::vector<std::size_t> infeasibleReqs = d_bestSol.d_reqInfeasible;

    // Add the infeasible requests to the unassigned requests
    d_bestSol.d_reqUnassigned.insert(d_bestSol.d_reqUnassigned.end(), infeasibleReqs.begin(), infeasibleReqs.end());

    // Copy the best solution to the temporary solution and repair it without considering service areas
    // Store the result in all solutions
    d_tempSol = d_bestSol;
    d_settings.d_restricedServiceArea = false;
    repairGreedyLocal(true, false);
    d_bestObjVal = d_tempSol.d_cost;
    d_bestSol = d_tempSol;
    d_curSol = d_tempSol;

    // Perform nIterations iterations for insertion of the infeasible requests
    for (std::size_t i = 0; i < static_cast<std::size_t>(d_settings.d_nIterations); ++i) {
        // Randomly shuffle the infeasible requests
        std::shuffle(infeasibleReqs.begin(), infeasibleReqs.end(), d_gen);
        // Remove the first nNeighborhood requests from the unassigned requests
        size_t nNeighborhood = std::min(d_settings.d_maxNeighborhoodSize, infeasibleReqs.size());
        for (std::size_t j = 0; j < nNeighborhood; ++j) {
            d_tempSol.removeNode(infeasibleReqs[j]);
        }
        // Repair the solution
        setOperationIdx();
        performRepair();

        // Evaluate the solution
        evaluateSolution(i+d_settings.d_nIterations);
        updateOpWeights(); 
    }
    d_settings.d_restricedServiceArea = true;
}