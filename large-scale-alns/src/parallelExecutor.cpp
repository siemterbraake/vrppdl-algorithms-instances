#include "parallelExecutor.h"

// TODO: Add loading of instance files from the data folder
ParallelExecutor::ParallelExecutor(std::string idRun){
    // Load the settings, instance and graph
    if (d_settings.load("settings.txt"))
        std::cout << "Settings loaded succesfully\n";
    else {
        std::cout << "ERROR: Problem with loading the settings file, aborting... \n";
        exit(1);
    };
    if (d_inst.load("../instances/" + d_settings.d_instName + ".txt")) {
        std::cout << "Instance loaded succesfully\n";
    }
    else {
        std::cout << "ERROR: Problem with loading the instance file, aborting... \n";
        exit(1);
    };

    // When working with fixed service regions, load the service areas
    if (d_settings.d_restricedServiceArea) {
        d_inst.addAllocatedDepots("tall.csv");
    }

    d_graph.load(d_inst, d_settings);

    // If nThreads is equal to 0, set the recommended number of threads
    if (d_settings.d_nThreads == 0) {
        d_settings.d_nThreads = std::thread::hardware_concurrency();
    }
    // If nThreads is larger than 1, set verbose to false
    if (d_settings.d_nThreads > 1) {
        d_settings.d_verbose = false;
    }

    // Resize the list of metaheuristics
    d_metaheuristics.reserve(d_settings.d_nThreads); 
    d_threads.resize(d_settings.d_nThreads);

    // Create the metaheuristic objects 
    for (std::size_t i = 0; i < d_settings.d_nThreads; i++){
        // Ofset the seed for each thread
        std::size_t seed =d_settings.d_randomSeed + i;
        d_metaheuristics.push_back(Metaheuristic(d_settings, d_inst, d_graph, "output/", idRun, seed));
        d_metaheuristics[i].findInitialSolution();
        d_metaheuristics[i].updateSubproblems();
    }
    distributeBestSolution();
}

void ParallelExecutor::run() {
    /*
    Starts a parallel metaheuristic run with the given number of threads.
    */
    auto runStart = std::chrono::steady_clock::now();
    std::cout << "\nStarting parallel execution with " << d_settings.d_nThreads << " threads...\n";
    for (std::size_t i = 0; i < d_settings.d_nThreadLoops; i++){
        auto loopStart = std::chrono::steady_clock::now();
        // Start the threads
        for (std::size_t j = 0; j < d_metaheuristics.size(); j++){
            d_threads[j] = std::thread(&Metaheuristic::run, &d_metaheuristics[j]);
        }
        // Wait until all threads are done
        for (std::size_t j = 0; j < d_threads.size(); j++){
            d_threads[j].join();
            d_metaheuristics[j].d_parallelIter += 1;
        }
        auto end = std::chrono::steady_clock::now();
        std::cout << "Loop " << i+1 << "/" << d_settings.d_nThreadLoops << " finished in " << std::chrono::duration_cast<std::chrono::duration<double>>(end - loopStart).count() << " seconds\n";
        // Find the thread with the best solution
        distributeBestSolution();
        // Print the results
        std::cout << "Best solution found by thread " << d_bestThread << " with objective value: " << d_metaheuristics[d_bestThread].d_bestObjVal << "\n";
        std::cout << "Change-over time: " << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - end).count() << " seconds\n";
    }

    if (d_settings.d_restricedServiceArea) {
        // Patch the infeasible requests on all threads
        for (std::size_t i = 0; i < d_threads.size(); i++){
            d_threads[i] = std::thread(&Metaheuristic::patchInfeasibleRequests, &d_metaheuristics[i]);
        }
        // Wait until all threads are done
        for (std::size_t i = 0; i < d_threads.size(); i++){
            d_threads[i].join();
            d_metaheuristics[i].d_parallelIter += 1;
        }
        distributeBestSolution();
    }
    // Print the results
    std::cout << "Best solution for insertion the infeasible requests found by thread " << d_bestThread << " with objective value: " << d_metaheuristics[d_bestThread].d_bestObjVal << "\n";
    auto runEnd = std::chrono::steady_clock::now();
    std::cout << "\nParallel Metaheuristic finished. Found solution with objective " << d_metaheuristics[d_bestThread].d_bestObjVal <<
              " in " << std::chrono::duration_cast<std::chrono::duration<double>>(runEnd - runStart).count() << " seconds\n";
}
void ParallelExecutor::runSubproblem(std::size_t subproblem) {
    /*
    Starts a parallel metaheuristic run with the given number of threads.
    */
    // Start the threads
    for (std::size_t i = 0; i < d_metaheuristics.size(); i++){
        d_threads[i] = std::thread(&Metaheuristic::runSubproblem, &d_metaheuristics[i], subproblem);
    }
    // Wait until all threads are done
    for (std::size_t i = 0; i < d_threads.size(); i++){
        d_threads[i].join();
        d_metaheuristics[i].d_parallelIter += 1;
    }
    distributeBestSolution();
}

void ParallelExecutor::distributeBestSolution() {
    /*
    Checks the thread with the best solutions and distributes best solution to all threads
    */
    // Find the thread with the best solution
    d_bestThread = 0;
    for (std::size_t i = 0; i < d_metaheuristics.size(); i++){
        if (d_metaheuristics[i].d_bestObjVal < d_metaheuristics[d_bestThread].d_bestObjVal){
            d_bestThread = i;
        }
    }
    d_bestObjVal = d_metaheuristics[d_bestThread].d_bestObjVal;
    d_metaheuristics[d_bestThread].updateSubproblems();
    // Copy the best solution to all threads
    for (std::size_t i = 0; i < d_metaheuristics.size(); i++){
        if (i==d_bestThread) {
            continue;
        }
        d_metaheuristics[i].d_curSol = d_metaheuristics[d_bestThread].d_bestSol;
        d_metaheuristics[i].d_tempSol = d_metaheuristics[d_bestThread].d_bestSol;
        d_metaheuristics[i].d_bestSol = d_metaheuristics[d_bestThread].d_bestSol;
        d_metaheuristics[i].d_bestObjVal = d_metaheuristics[d_bestThread].d_bestObjVal;
    }
}

const std::vector<std::vector<std::size_t>>& ParallelExecutor::getSubproblems() const {
    /*
    returns the subproblems
    */
    return d_metaheuristics[0].getSubproblems();
}

float ParallelExecutor::calcSubproblemCost(std::size_t idSubproblem) const {
    /*
    Returns the subproblem cost
    */
    return d_metaheuristics[0].calcSubproblemCost(idSubproblem);
}

std::vector<std::vector<float>> ParallelExecutor::getSubproblemEmbeddings(std::size_t idSubproblem) const {
    /*
    Return the subproblem embeddings
    */
    return d_metaheuristics[0].getSubproblemEmbeddings(idSubproblem);
}

void ParallelExecutor::writeLogs(float timeElapsed, bool error, bool python) const {
    /*
    Writes the logs
    */
    d_metaheuristics[d_bestThread].writeLogs(timeElapsed, error, python);
}



    