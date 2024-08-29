#include "result.hpp"

Result::Result() {
    /*
    Default constructor
    */
    d_instName = "";
    d_alnsInteger = false;
    d_exactInteger = false;
    d_lbALNS = 0.0;
    d_lbExact = 0.0;
    d_ubMPExact = 0.0;
    d_ubMPALNS = 0.0;
    d_nPricingALNS = 0;
    d_nPricingALNSExact = 0;
    d_nPricingExact = 0;
    d_nCustPerRoutePricing = 0;
    d_tMPALNS = 0;
    d_tMPExact = 0;
}

bool Result::isValid(double tolerance) {
    /*
    Checks if the results are valid
    */
    // If there is an exact lb, it must be equal to alns lb
    if (d_lbExact > 0 && std::abs(d_lbExact - d_lbALNS) > tolerance) {
        return false;
    }
    return true;
}

void Result::print() {
    /*
    Prints the result to the console
    */
    std::cout << "\n-- Instance " << d_instName << " solved --\n";
    std::cout << "Exact algorithm performed " << d_nPricingExact << " pricing loops in " << d_tMPExact << " seconds.\n";
    std::cout << "ALNS algorithm performed " << d_nPricingALNS << " alns pricing loops and " << d_nPricingALNSExact << " exact pricing loops in " << d_tMPALNS << " seconds.\n";
}