#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <chrono>

#include "result.hpp"
#include "solver.hpp"

int main()
{
    std::vector<std::string> paths;
    std::vector<Result> results;
    std::string path = "../instances/small/";

    // Store the current date and time as a string
    std::time_t now_c = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%y%m%d%H%M%S");
    std::string idRun = ss.str();

    unsigned short nError = 0;

    for (const auto& entry : std::filesystem::directory_iterator(path)) {
        // If it is a folder, continue
        if (entry.is_directory()) {
            continue;
        }
        paths.push_back(entry.path().string()); 
    }

    // Sort alphabetically
    std::sort(paths.begin(), paths.end());

    // Reserve space for the results
    results.reserve(paths.size());

    // Create a result file
    std::filesystem::create_directory("output/logs/");
    std::ofstream file("output/logs/"+idRun+".csv");
    file << "Instance, Exact Pricing, ALNS Pricing, ALNS Exact, Exact LB, Exact MP UB, ALNS LB, ALNS MP UB, Exact MP time, ALNS MP time\n";

    // Solve all instances
    for (unsigned short iInst = 0; iInst < paths.size(); iInst++)
    {
        Solver solver(paths[iInst]);
        solver.solvePricing("exact");
        solver.solvePricing("alns");
        solver.printResult();
        results.push_back(solver.getResult());
        // Write the results to a csv file
        file << results[iInst].d_instName << ","
             << results[iInst].d_nPricingExact << "," 
             << results[iInst].d_nPricingALNS << "," 
             << results[iInst].d_nPricingALNSExact << "," 
             << results[iInst].d_lbExact << "," 
             << results[iInst].d_ubMPExact << "," 
             << results[iInst].d_lbALNS << "," 
             << results[iInst].d_ubMPALNS <<  "," 
             << results[iInst].d_tMPExact << "," 
             << results[iInst].d_tMPALNS << "\n";
        file.flush();
        if (!solver.verifyResult())
        {
            nError++;
        }
    }
    file.close();

    // Print the results
    std::cout << "Number of errors: " << nError << std::endl;
    return 0;
}