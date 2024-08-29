/*
Main function for the metaheuristic, which imports an instance and a settings file, and runs the metaheuristic.

Written by: Siem ter Braake, 2024
*/

#include "parallelExecutor.h"
#include <chrono>
#include <iomanip>

#include <iostream>

int main () {
    // Get current time to set the id of the run
    std::time_t now_c = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%y%m%d%H%M%S");
    std::string idRun = ss.str();
    
    // Initialize metaheuristic
    ParallelExecutor exec(idRun);

    // Run metaheuristic
    exec.run();

    // Write logs
    exec.writeLogs(0, false, false);

    return 0;
}