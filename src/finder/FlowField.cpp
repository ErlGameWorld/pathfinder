#include "FlowField.h"
#include "../common/VizMacros.h"
#include <queue>
#include <algorithm>

FlowField::FlowField(HexMap& m) : map(m), currentEnd(-9999, -9999) {
    parentMap.resize(map.width * map.height, -1);
}

void FlowField::reset() {
    std::fill(parentMap.begin(), parentMap.end(), -1);
    built = false;
    currentEnd = Hex(-9999, -9999);
}

void FlowField::onMapUpdate(const std::vector<Hex>& changedHexes) {
    built = false; 
}

std::vector<Hex> FlowField::findPath(Hex start, Hex end) {
    if (!map.isWalkable(start) || !map.isWalkable(end)) return {};
    
    // Check if we need to rebuild field
    if (!built || end != currentEnd) {
        std::fill(parentMap.begin(), parentMap.end(), -1);
        currentEnd = end;
        
        std::queue<int> q;
        auto getIdx = [&](const Hex& h) {
            int col = h.q + (h.r - (h.r & 1)) / 2;
            int row = h.r;
            if (col < 0 || col >= map.width || row < 0 || row >= map.height) return -1;
            return row * map.width + col;
        };
        
        int endIdx = getIdx(end);
        if (endIdx != -1) {
            parentMap[endIdx] = endIdx; // Sentinel
            q.push(endIdx);
            
            while(!q.empty()) {
                int currIdx = q.front();
                q.pop();
                
                int r = currIdx / map.width;
                int c = currIdx % map.width;
                int q_ax = c - (r - (r & 1)) / 2;
                Hex currHex(q_ax, r);
                
                for (const auto& dir : HEX_DIRS) {
                    Hex next = currHex + dir;
                    if (!map.isWalkable(next)) continue;
                    
                    int nextIdx = getIdx(next);
                    if (nextIdx != -1 && parentMap[nextIdx] == -1) {
                        parentMap[nextIdx] = currIdx; // Point towards curr (which is closer to end)
                        q.push(nextIdx);
                    }
                }
            }
        }
        built = true;
    }
    
    std::vector<Hex> path;
    
    auto getIdx = [&](const Hex& h) {
        int col = h.q + (h.r - (h.r & 1)) / 2;
        int row = h.r;
        if (col < 0 || col >= map.width || row < 0 || row >= map.height) return -1;
        return row * map.width + col;
    };
    
    int currIdx = getIdx(start);
    if (currIdx == -1 || parentMap[currIdx] == -1) return {}; 
    
    VIZ_START(); 
    
    int safety = 0;
    while (safety++ < 100000) {
        int r = currIdx / map.width;
        int c = currIdx % map.width;
        int q_ax = c - (r - (r & 1)) / 2;
        Hex h(q_ax, r);
        
        path.push_back(h);
        VIZ_LOG(h);
        
        if (h == end) break;
        
        int nextIdx = parentMap[currIdx];
        if (nextIdx == currIdx || nextIdx == -1) break; 
        currIdx = nextIdx;
    }
    
    return path;
}
