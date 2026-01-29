#pragma once

#include "IFinder.h"
#include "AStar.h"
#include <unordered_map>
#include <vector>
#include <set>

struct AbstractEdge { Hex target; int cost; bool isTemporary; };
struct Cluster { int id; std::vector<Hex> entrances; };
struct PortalInfo { Hex entranceNode; std::vector<Hex> pixels; int clusterId; };

class DHPAStar : public IFinder {
    HexMap& map;
    AStar localSolver;
    std::unordered_map<int, Cluster> clusters;
    std::unordered_map<Hex, std::vector<AbstractEdge>> abstractGraph; 
    std::vector<Hex> modifiedNodes; 

public:
    DHPAStar(HexMap& m);

    // IFinder impl
    std::string getName() const override { return "DHPAStar"; }
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
