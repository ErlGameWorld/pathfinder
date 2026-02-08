#include "DHPAJps.h"
#include <queue>
#include <unordered_set>
#include <limits>

DHPAJps::DHPAJps(HexMap& m) : map(m), localSolver(m) {
    buildGraph();
}

std::vector<Hex> DHPAJps::findPath(Hex start, Hex end) {
    // 简化实现：直接使用JPS寻路
    return localSolver.findPath(start, end);
}

void DHPAJps::reset() {
    buildGraph();
}

void DHPAJps::onMapUpdate(const std::vector<Hex>& changedHexes) {
    // 简化实现：重新构建图
    for (const auto& h : changedHexes) {
        modifiedNodes.push_back(h);
    }
    // 实际应该只更新受影响的簇
    if (!modifiedNodes.empty()) {
    }
}

void DHPAJps::buildGraph() {
    // 简化实现：JPS不需要预构建图

}

void DHPAJps::addEdge(Hex u, Hex v, int cost, bool isTemporary) {
    // 简化实现
}

void DHPAJps::refreshCluster(int cId) {
    // 简化实现
}

void DHPAJps::connectToGraph(Hex node) {
    // 简化实现
}

void DHPAJps::cleanupDirtyNodes() {
    modifiedNodes.clear();
}
