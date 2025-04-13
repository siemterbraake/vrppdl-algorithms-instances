#include "settings.h"

Settings::Settings() {
    /* 
    Constructor for the settings struct, sets default values.
    */
    d_instName = "";
    d_distMetric = "euclidean";
    d_nThreads = 1;
    d_nConstructionTries = 1;
    d_nDestroyOps = 1;
    d_nRepairOps = 1;
    d_nNeighborConsider = 1;
    d_maxNeighborhoodSize = 1;
    d_subproblemSize = 1;
    d_nIterations = 1;
    d_nCheckFeasible = 1;
    d_nThreadLoops = 1;
    d_randomSeed = 0;
    d_sortType = 0;
    d_verbose = false;
    d_local = false;
    d_restricedServiceArea = false;
    d_writeFiles = false;
    d_noiseLevel = 0.0;
    d_alnsAlpha = 0.1f;
    d_excessiveRoutePenalty = 1000;
    d_nUnimproveMax = 1000;
    d_pThresholdMax = 0.0f;
}

void parseKeyValue(const std::string& line, std::unordered_map<std::string, std::string>& keyValueMap) {
    /*
    Function to parse key-value pairs from a line in the settings file.
    */
    std::istringstream iss(line);
    std::string key, value;
    if (std::getline(iss, key, '=') && std::getline(iss, value)) {
        keyValueMap[key] = value;
    }
}

bool Settings::load(const std::string path_settings) {
    /*
    Loads the settings and stores them in the settings struct.
    */
    std::ifstream file(path_settings);
    std::string line;
    std::unordered_map<std::string, std::string> keyValueMap;

    auto setValue = [&](const std::string& key, auto& variable) {
        if (keyValueMap.find(key) != keyValueMap.end()) {
            std::istringstream iss(keyValueMap.at(key));
            if (iss >> variable) {
                return true;
            }
        }
        std::cout << "ERROR: " << key << " not found in settings file.\n";
        return false;        
    };

    while (std::getline(file, line)) {
        parseKeyValue(line, keyValueMap);
    }

    // Load parameters into setting struct
    if (setValue("instName", d_instName) &&
        setValue("distMetric", d_distMetric) &&
        setValue("writeFiles", d_writeFiles) &&
        setValue("verbose", d_verbose) &&
        setValue("nThreads", d_nThreads) &&
        setValue("nConstructionTries", d_nConstructionTries) &&
        setValue("nDestroyOps", d_nDestroyOps) &&
        setValue("nRepairOps", d_nRepairOps) &&
        setValue("local", d_local) &&
        setValue("nNeighborConsider", d_nNeighborConsider) &&
        setValue("maxNeighborhoodSize", d_maxNeighborhoodSize) &&
        setValue("subproblemSize", d_subproblemSize) &&
        setValue("restrictedServiceArea", d_restricedServiceArea) &&
        setValue("nIterations", d_nIterations) &&
        setValue("nCheckFeasible", d_nCheckFeasible) &&
        setValue("nThreadLoops", d_nThreadLoops) &&
        setValue("randomSeed", d_randomSeed) &&
        setValue("sortType", d_sortType)  &&
        setValue("noiseLevel", d_noiseLevel) &&
        setValue("alnsAlpha", d_alnsAlpha) &&
        setValue("nUnimproveMax", d_nUnimproveMax) &&
        setValue("pThresholdMax", d_pThresholdMax) && 
        setValue("excessiveRoutePenalty", d_excessiveRoutePenalty)) {
            return true;
        }

    return false;    
}
