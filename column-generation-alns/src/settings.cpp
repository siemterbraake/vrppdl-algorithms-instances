#include "settings.hpp"

Settings::Settings() {
    /* 
    Constructor for the settings struct, sets default values.
    */
    d_distMetric = "euclidean";
    d_nConstructionTries = 1;
    d_nDestroyOps = 1;
    d_nRepairOps = 1;
    d_maxNeighborhoodSize = 1;
    d_nALNSInitIterationsPerNode = 1;
    d_nALNSIterationsPerNode = 1;
    d_nCheckFeasible = 1;
    d_bigM = 1.0;
    d_tolerance = 0.0;
    d_randomSeed = 0;
    d_sortType = 0;
    d_verbose = false;
    d_writeFiles = false;
    d_simulatedAnnealing = false;
    d_alnsInit = false;
    d_verifyExact = false;
    d_writeSols = false;
    d_noiseLevel = 0.0;
    d_alnsAlpha = 0.1f;
    d_tempStartSA = 1;
    d_tempEndSA = 1;
    d_nColsMaxExact = 1;
    d_nColsMaxALNS = 1;
    d_nSolsInit = 1;
    d_timeLimit = 600;
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
    if (setValue("distMetric", d_distMetric) &&
        setValue("alnsInit", d_alnsInit) &&
        setValue("verifyExact", d_verifyExact) &&
        setValue("writeFiles", d_writeFiles) &&
        setValue("verbose", d_verbose) &&
        setValue("nConstructionTries", d_nConstructionTries) &&
        setValue("nDestroyOps", d_nDestroyOps) &&
        setValue("nRepairOps", d_nRepairOps) &&
        setValue("bigM", d_bigM) &&
        setValue("tolerance", d_tolerance) &&
        setValue("maxNeighborhoodSize", d_maxNeighborhoodSize) &&
        setValue("nALNSIterationsPerNode", d_nALNSIterationsPerNode) &&
        setValue("nALNSInitIterationsPerNode", d_nALNSInitIterationsPerNode) &&
        setValue("nCheckFeasible", d_nCheckFeasible) &&
        setValue("writeSols", d_writeSols) &&
        setValue("nColsMaxExact", d_nColsMaxExact) &&
        setValue("nColsMaxALNS", d_nColsMaxALNS) &&
        setValue("nSolsInit", d_nSolsInit) &&
        setValue("nRestartALNS", d_nRestartALNS) &&
        setValue("randomSeed", d_randomSeed) &&
        setValue("sortType", d_sortType)  &&
        setValue("noiseLevel", d_noiseLevel) &&
        setValue("alnsAlpha", d_alnsAlpha) &&
        setValue("simulatedAnnealing", d_simulatedAnnealing) &&
        setValue("timeLimit", d_timeLimit) &&
        setValue("tempStartSA", d_tempStartSA) &&
        setValue("tempEndSA", d_tempEndSA)) {
            return true;
        }

    return false;    
}
