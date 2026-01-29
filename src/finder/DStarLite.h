#pragma once
#include "IFinder.h"
#include "../map/HexMap.h"
#include <vector>
#include <queue>
#include <unordered_map>

class DStarLite : public IFinder {
    HexMap& map;

    struct Key {
        double k1, k2;
        bool operator<(const Key& other) const {
            if (k1 != other.k1) return k1 < other.k1;
            return k2 < other.k2;
        }
        bool operator>(const Key& other) const {
            if (k1 != other.k1) return k1 > other.k1;
            return k2 > other.k2;
        }
    };

    struct Node {
        double g = 2147483647.0;
        double rhs = 2147483647.0;
    };

    std::vector<Node> nodeData;
    typedef std::pair<Key, int> PQPair;
    std::priority_queue<PQPair, std::vector<PQPair>, std::greater<PQPair>> openList;
    
    Hex startHex, goalHex;
    double km = 0;
    
    // Helper to track open list membership roughly (or use set for precise remove/update, but PQ with lazy deletion is common)
    // Actually D* Lite requires updating keys. Standard PQ doesn't support decrease-key efficiently.
    // Lazy deletion approach: push new key, check validity on pop.
    // We store 'version' or check consistency.
    
    // For D* Lite, we need to know if a node is in Open list to update it.
    std::vector<bool> inOpen; 
    
public:
    DStarLite(HexMap& m);
    std::string getName() const override { return "DStarLite"; }
    std::vector<Hex> findPath(Hex start, Hex end) override;
    void reset() override;
    void onMapUpdate(const std::vector<Hex>& changedHexes) override;

private:
    int getIdx(Hex h);
    Hex getHex(int idx);
    Key calculateKey(int idx);
    void updateVertex(int idx);
    void computeShortestPath();
    double heuristic(Hex a, Hex b);
    double cost(int u, int v);
};
