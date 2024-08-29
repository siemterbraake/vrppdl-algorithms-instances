/*
The result struct stores the results of the algorithms.

Written by: Siem ter Braake, 2024
*/

#ifndef RESULT_H
#define RESULT_H

#include <iostream>

struct Result {
    std::string d_instName;             // Instance name
    double d_tMPALNS;                   // Time to run the master problem in the ALNS algorithm
    double d_tMPExact;                  // Time to run the MIP algorithm
    double d_ubMPExact;                 // Upper bound of the exact master problem
    double d_ubMPALNS;                  // Upper bound of the master problem with ALNS
    double d_lbALNS;                    // Lower bound of the ALNS algorithm
    double d_lbExact;                   // Lower bound of the MIP algorithm
    unsigned short d_nPricingALNS;      // Number of pricing problems solved by the ALNS algorithm
    unsigned short d_nPricingALNSExact; // Number of pricing problems solved exactly by the ALNS algorithm
    unsigned short d_nPricingExact;     // Number of pricing problems solved by the exact algorithm
    unsigned short d_nCustPerRoutePricing; // Number of customers per route in the pricing problem
    bool d_alnsInteger;                 // Whether the ALNS solution is integer
    bool d_exactInteger;                // Whether the exact solution is integer

    Result();
    void print();
    bool isValid(double tolerance);
};

#endif // RESULT_H