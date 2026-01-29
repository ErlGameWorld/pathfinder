#include "DStarLite.h"
#include "../common/VizMacros.h"
#include <algorithm>
#include <cmath>
#include <iostream>

// Use a large value for infinity
const double INF = 2147483647.0;

DStarLite::DStarLite(HexMap& m) : map(m) {
    nodeData.resize(map.width * map.height);
    inOpen.resize(map.width * map.height, false);
}

void DStarLite::reset() {
    nodeData.clear();
    nodeData.resize(map.width * map.height);
    inOpen.clear();
    inOpen.resize(map.width * map.height, false);
    while(!openList.empty()) openList.pop();
    km = 0;
    startHex = Hex(-9999, -9999);
    goalHex = Hex(-9999, -9999);
}

int DStarLite::getIdx(Hex h) {
    int col = h.q + (h.r - (h.r & 1)) / 2;
    int row = h.r;
    if (col < 0 || col >= map.width || row < 0 || row >= map.height) return -1;
    return row * map.width + col;
}

Hex DStarLite::getHex(int idx) {
    int r = idx / map.width;
    int c = idx % map.width;
    int q = c - (r - (r & 1)) / 2;
    return Hex(q, r);
}

double DStarLite::heuristic(Hex a, Hex b) {
    return a.distance(b); // Unit cost = 1
}

double DStarLite::cost(int u, int v) {
    Hex hU = getHex(u);
    Hex hV = getHex(v);
    if (!map.isWalkable(hU) || !map.isWalkable(hV)) return INF;
    return 1.0;
}

DStarLite::Key DStarLite::calculateKey(int idx) {
    double minVal = std::min(nodeData[idx].g, nodeData[idx].rhs);
    return {minVal + heuristic(getHex(idx), startHex) + km, minVal};
}

void DStarLite::updateVertex(int idx) {
    if (idx != getIdx(goalHex)) {
        double minRhs = INF;
        Hex u = getHex(idx);
        for (const auto& dir : HEX_DIRS) {
            Hex succ = u + dir;
            int sIdx = getIdx(succ);
            if (sIdx != -1) {
                minRhs = std::min(minRhs, cost(idx, sIdx) + nodeData[sIdx].g);
            }
        }
        nodeData[idx].rhs = minRhs;
    }
    
    // Lazy update: We don't remove from PQ, just add new one.
    // However, D* Lite logic requires checking if it's in Open to update or remove.
    // Since we use lazy PQ, we just push. The pop loop handles stale entries.
    // Stale check: pop u, re-calc key. If key differs from popped key, discard?
    // Actually, D* Lite usually implies strict PQ handling.
    // Let's rely on redundant pushes and checking consistency on pop.
    
    // Check if g != rhs
    bool inconsistent = (nodeData[idx].g != nodeData[idx].rhs);
    
    if (inconsistent) {
        openList.push({calculateKey(idx), idx});
    }
}

void DStarLite::computeShortestPath() {
    while (!openList.empty()) {
        PQPair top = openList.top();
        Key kOld = top.first;
        int uIdx = top.second;
        
        // Lazy check: Is this key valid?
        // If we popped it, let's see if we need to process it.
        // Re-calculate key
        Key kNew = calculateKey(uIdx);
        
        // If current key is strictly less than what we have in node (meaning we found better?)
        // Or if node state matches what was pushed?
        // Simpler: Just compare keys.
        
        Key kStart = calculateKey(getIdx(startHex));
        if (kOld < kStart || nodeData[getIdx(startHex)].rhs != nodeData[getIdx(startHex)].g) {
             // Continue processing
        } else {
             // Start is consistent and top key >= start key
             break;
        }

        openList.pop();
        
        Hex u = getHex(uIdx);
        VIZ_LOG(u); // Log expansion

        if (kOld < kNew) {
            openList.push({kNew, uIdx});
        } else if (nodeData[uIdx].g > nodeData[uIdx].rhs) {
            nodeData[uIdx].g = nodeData[uIdx].rhs;
            for (const auto& dir : HEX_DIRS) {
                Hex pred = u + dir;
                int pIdx = getIdx(pred);
                if (pIdx != -1) updateVertex(pIdx);
            }
        } else {
            nodeData[uIdx].g = INF;
            updateVertex(uIdx); // Update self
            for (const auto& dir : HEX_DIRS) {
                Hex pred = u + dir;
                int pIdx = getIdx(pred);
                if (pIdx != -1) updateVertex(pIdx);
            }
        }
    }
}

std::vector<Hex> DStarLite::findPath(Hex start, Hex end) {
    bool newGoal = (goalHex != end);
    bool newStart = (startHex != start); // Moved start

    if (startHex.q == -9999 || newGoal) {
        // Full Reset / Init
        reset();
        startHex = start;
        goalHex = end;
        
        int goalIdx = getIdx(goalHex);
        if (goalIdx == -1) return {};
        
        nodeData[goalIdx].rhs = 0;
        openList.push({calculateKey(goalIdx), goalIdx});
        
        computeShortestPath();
    } else if (newStart) {
        // Start moved (Robot moved)
        // D* Lite optimization: Update km
        km += heuristic(startHex, start);
        startHex = start;
        // No need to re-compute entire graph unless edge costs changed (handled in onMapUpdate)
        // Just verify start consistency?
        // D* Lite logic: computeShortestPath() will run until start is consistent.
        computeShortestPath();
    }
    
    // Extract Path
    std::vector<Hex> path;
    Hex curr = startHex;
    int currIdx = getIdx(curr);
    
    if (nodeData[currIdx].g == INF) return {}; // No path
    
    path.push_back(curr);
    int safety = 0;
    while (curr != goalHex && safety++ < 5000) {
        // Move to neighbor with min (c + g)
        double minCost = INF;
        Hex next = curr;
        
        for (const auto& dir : HEX_DIRS) {
            Hex n = curr + dir;
            int nIdx = getIdx(n);
            if (nIdx != -1) {
                double c = cost(currIdx, nIdx);
                if (c != INF && nodeData[nIdx].g < INF) {
                    double val = c + nodeData[nIdx].g;
                    if (val < minCost) {
                        minCost = val;
                        next = n;
                    }
                }
            }
        }
        
        if (next == curr) break; // Stuck
        curr = next;
        currIdx = getIdx(curr);
        path.push_back(curr);
    }
    
    return path;
}

void DStarLite::onMapUpdate(const std::vector<Hex>& changedHexes) {
    // Edge costs changed.
    // For each changed hex, its incident edges changed cost.
    // Specifically, if hex becomes obstacle, cost to/from it becomes INF.
    // If it becomes free, cost becomes 1.
    
    // Note: D* Lite assumes we detect edge changes.
    // A changed Hex affects 6 incoming edges and 6 outgoing edges.
    
    for (const Hex& h : changedHexes) {
        int idx = getIdx(h);
        if (idx == -1) continue;
        
        // Update neighbors
        for (const auto& dir : HEX_DIRS) {
            Hex n = h + dir;
            int nIdx = getIdx(n);
            if (nIdx != -1) {
                updateVertex(idx); // Update self based on neighbors
                updateVertex(nIdx); // Update neighbor based on self
            }
        }
    }
    
    computeShortestPath();
}
