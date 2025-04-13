/*
The pricer performs the column generation algorithm to solve the pricing problem in an exact manner.

Written by: Casper Bazelmans, 2024
*/

#ifndef PRICER_H
#define PRICER_H

#include "instance.hpp"
#include "lpSolution.hpp"
#include "label.hpp"
#include "graph.hpp"

#include <vector>
#include <deque>
#include <set>


struct Pricer
{
    const LPSolution& d_sol;
    const Settings& d_settings;
    const Instance& d_inst;
    const Graph& d_graph;

    std::vector<std::deque<Label>> d_pathsUnprocessed;   // Save paths end in node, since it's easy for dominance check
    std::vector<std::deque<Label>> d_pathsProcessed;     // Save paths end in node, since it's easy for dominance check
    std::vector<std::vector<int>> d_durations;
    std::vector<std::vector<double>> d_reducedCosts;
    std::vector<std::vector<unsigned char>> d_slsToDepot;
    std::vector<std::vector<unsigned char>> d_slsFromDepot;

    std::set<std::vector<unsigned short>> d_routesUnique;
    std::deque<Label> d_pathsComplete;

    unsigned short d_idDepot;
    unsigned short d_idVehType;
    unsigned short d_nColsFound;
    unsigned short d_ctrNewBest;

    double d_reducedCostBest;

    Pricer(const LPSolution& sol, const Settings& settings, const Instance& inst, const Graph& graph);
    void setSubProblem(unsigned short idDepot, unsigned short idVehType);
    void solveExact(std::atomic<bool>& stopFlag, unsigned short nColsMax);
    std::deque<Label> getCompleteLabels();
    void initEmpty();
    void updateDurationsMatrix();
    void updateCostMatrix();
    void processAvailableSLs();
    Label createStartLabel();
    void extendLabel(const Label &labelFrom, const Node &nodeTo, std::atomic<bool>& stopFlag);
    void checkCompleteRoute(const Label &path);

    inline void removeLabelsDominatedBy(const Label &pathPartial, std::atomic<bool>& stopFlag)
    {
        int ctrRemove = 0;

        for (unsigned short idLabel = 0; idLabel < d_pathsProcessed[pathPartial.d_idNode].size(); idLabel++)
        {
            if (stopFlag.load())
            {
                return;
            }
            if (dominates(pathPartial, d_pathsProcessed[pathPartial.d_idNode][idLabel]))
            {
                d_pathsProcessed[pathPartial.d_idNode].erase(
                    d_pathsProcessed[pathPartial.d_idNode].begin() + idLabel - ctrRemove
                );
                ctrRemove++;
            }
        }
    }

    inline bool isLabelDominated(const Label &pathPartial, std::atomic<bool>& stopFlag)
    {
        for (unsigned short idLabel = 0; idLabel < d_pathsProcessed[pathPartial.d_idNode].size(); idLabel++)
        {
            if (stopFlag.load())
            {
                return true;
            }
            if (dominates(d_pathsProcessed[pathPartial.d_idNode][idLabel], pathPartial))
            {
                return true;
            }
        }
        return false;
    }
    
    // Pricer object cannot be moved
    Pricer& operator=(const Pricer&) = delete;
};


#endif // PRICER_H