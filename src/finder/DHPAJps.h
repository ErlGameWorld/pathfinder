#pragma once

#include "IFinder.h"
#include "JPS.h"
#include <unordered_map>
#include <vector>
#include <set>

// Reuse structure definitions if possible, but they were local to DHPAStar.cpp or private?
// In DHPAStar.h they are struct definitions outside class but in header.
// So we can reuse them if we include DHPAStar.h or redefine.
// DHPAStar.h defines AbstractEdge, Cluster, PortalInfo.

#include "DHPAStar.h"

class DHPAJps : public IFinder {
    HexMap& map;
    JPS localSolver; // Uses JPS instead of AStar
    
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
    void connectToGraph(Hex node);
    void cleanupDirtyNodes();
};
