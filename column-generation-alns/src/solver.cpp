#include "solver.hpp"

Solver::Solver(std::string path)
{
    /*
    Constructor for the solver class, initializes the relevant objects
    */
    if (d_settings.load("settings.txt")) {
        if (d_settings.d_verbose)
            std::cout << "Settings loaded succesfully\n";
    }
    else 
    {
        std::cout << "ERROR: Problem with loading the settings file, aborting... \n";
        exit(1);
    };
    if (d_inst.load(path)) {
        if (d_settings.d_verbose)
            std::cout << "\nInstance loaded succesfully\n";
    }
    else 
    {
        std::cout << "ERROR: Problem with loading the instance file, aborting... \n";
        exit(1);
    };

    d_graph.load(d_inst, d_settings);

    // Settings sanity checks
    d_settings.d_maxNeighborhoodSize = std::min(d_settings.d_maxNeighborhoodSize, static_cast<unsigned short>(d_inst.d_nCustNodes - 2));
    d_result.d_instName = d_inst.d_name;
}

void Solver::solvePricing(std::string pricingMethod)
{
    /*
    Solve the instance using the exact pricing algorithm
    */

   // If pricing is set to exact, but verifyExact is not set, return
    if (pricingMethod == "exact" && !d_settings.d_verifyExact) {
        return;
    }

    auto timeStart = std::chrono::high_resolution_clock::now();

    MasterProblemGRB mp = MasterProblemGRB(d_settings, d_inst, d_graph);
    mp.initDummyVar();
    if (d_settings.d_alnsInit) {
        mp.initALNS();
    }

    // Create an atomic boolean to stop the function if the time limit is reached
    std::atomic<bool> stopFlag{false};

    // Set the pricing method as a function
    auto labelFunction = [&mp, &pricingMethod, &stopFlag, this]() {
        if (pricingMethod == "exact") {
            mp.labelExact(d_result, stopFlag);
        }
        else if (pricingMethod == "alns") {
            mp.labelALNS(d_result, stopFlag);
        }
    };

    // Run the label function asynchronously
    auto future = std::async(std::launch::async, labelFunction);

    // Wait for the function to complete or the time limit to be reached
    if (future.wait_for(std::chrono::seconds(d_settings.d_timeLimit)) == std::future_status::timeout) {
        std::cout << "Time limit reached. Terminating the function." << std::endl;
        stopFlag.store(true);
    } else {
        future.get(); // Retrieve the result (if any) to propagate exceptions
    }

    if (d_settings.d_verbose) {
        mp.printSolution();
    }

    // Stop timing
    auto timeEnd = std::chrono::high_resolution_clock::now();
    double computationTime = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(timeEnd - timeStart).count()) / 1000.0;
    if (pricingMethod == "exact") {
        d_result.d_tMPExact = computationTime;
    } else if (pricingMethod == "alns") {
        d_result.d_tMPALNS = computationTime;
    }
}

void Solver::printResult()
{
    /*
    Prints the result of the solver
    */
    d_result.print();
}

bool Solver::verifyResult()
{
    /*
    Verifies the result of the solver
    */
    return d_result.isValid(d_settings.d_tolerance);
}

Result Solver::getResult()
{
    /*
    Returns the result of the solver
    */
    return d_result;
}