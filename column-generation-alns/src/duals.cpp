#include "duals.hpp"

Duals::Duals(const Instance& inst, const Settings& settings) :
d_nVehicleTypes(inst.d_nVehTypes), d_nDepots(inst.d_nDepots) {
    /*
    Constructor for the dual variables struct, initializes the dual variables.
    */
    d_customer = std::vector<double>(inst.d_nCustNodes, settings.d_bigM);
    d_vehicle = std::vector<double>(inst.d_nVehTypes*inst.d_nDepots, 0.0);
    d_sl = std::vector<double>(inst.d_nSL, 0.0);
}

void Duals::clear() {
    /*
    Set all duals to zero.
    */
    std::fill(d_customer.begin(), d_customer.end(), 0.0);
    std::fill(d_vehicle.begin(), d_vehicle.end(), 0.0);
    std::fill(d_sl.begin(), d_sl.end(), 0.0);
}