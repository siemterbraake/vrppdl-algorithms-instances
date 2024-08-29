#include "lpSolution.hpp"

LPSolution::LPSolution(const Instance& inst, const Settings& settings) : 
d_objective(0.0), d_duals(Duals(inst, settings)) {}

bool LPSolution::isInteger() const {
    for (double val : d_variableValues) {
        if (std::abs(val - std::round(val)) > 1e-6) {
            return false;
        }
    }
    return true;
}