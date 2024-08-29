/*
The LP solution struct stores the results of the linear programming relaxation of the column generation algorithm.

Written by: Casper Bazelmans, 2024
*/

#ifndef LP_SOL_H
#define LP_SOL_H

#include <duals.hpp>
#include <instance.hpp>
#include <settings.hpp>
#include <alnsSolution.hpp>

#include <vector>


struct LPSolution
{
    double d_objective;

    std::vector<double> d_variableValues;
    std::vector<double> d_variableReducedCosts;

    Duals d_duals;
    LPSolution() {};
    LPSolution(const Instance& inst, const Settings& settings);
    bool isInteger() const;
};


#endif // LP_SOL_H