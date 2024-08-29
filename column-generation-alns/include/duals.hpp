/*
Struct for the dual variables of the optimization problem.

Written by: Siem ter Braake, 2024
*/

#ifndef DUALS_H
#define DUALS_H

#include <instance.hpp>
#include <settings.hpp>

#include <vector>

struct Duals
{
    std::vector<double> d_customer;
    std::vector<double> d_vehicle;
    std::vector<double> d_sl;

    unsigned short d_nVehicleTypes;
    unsigned short d_nDepots;

    Duals() {};
    Duals(const Instance& inst, const Settings& settings);
    void clear();
};

#endif // DUALS_H
