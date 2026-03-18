#pragma once

#include "IFinder.h"
#include "JPS.h"
#include "AStar.h"
#include <unordered_map>
#include <vector>
#include <set>
#include "DHPAStar.h" // For struct definitions

class DHPAJps : public IFinder {
    HexMap& map;
    JPS localSolver;     // Used for refinement (fast, unconstrained)
    AStar builderSolver; // Used for graph building (supports cluster constraints)
    
    // Copy of DHPAStar data structures
    std::unordered_map<int, Cluster> clusters;
    std::unordered_map<Hex, std::vector<AbstractEdge>> abstractGraph; 
    std::vector<Hex> modifiedNodes; 

public:
    DHPAJps(HexMap& m);

    // IFinder impl
    std::string getName() const override { return "DHPAJps"; }
    std::vector<Hex> findPath(Hex start, Hex end) override;
    void reset() override;
    void onMapUpdate(const std::vector<Hex>& changedHexes) override;
    void buildGraph() override;

private:
    void addEdge(Hex u, Hex v, int cost, bool isTemporary);
    void refreshCluster(int cId);
    void rebuildCluster(int cId);
    void connectToGraph(Hex node);
    void cleanupDirtyNodes();
};
