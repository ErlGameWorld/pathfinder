#include "FlowField.h"
#include "../common/VizMacros.h"
#include <queue>
#include <algorithm>

FlowField::FlowField(HexMap& m) : map(m), currentEnd(-9999, -9999) {
    size_t sz = (size_t)map.width * (size_t)map.height;
    parentMap.resize(sz, -1);
}

void FlowField::reset() {
    size_t sz = (size_t)map.width * (size_t)map.height;
    if (parentMap.size() != sz) {
        parentMap.clear();
        parentMap.resize(sz, -1);
    } else {
        std::fill(parentMap.begin(), parentMap.end(), -1);
    }
    
    // Clear queue properly
    std::queue<int> empty;
    std::swap(openQueue, empty);
    
    built = false;
    currentEnd = Hex(-9999, -9999);
}

void FlowField::onMapUpdate(const std::vector<Hex>& changedHexes) {
    // Invalidate everything for safety
    // Optimized incremental update is possible but complex for FlowField
    built = false; 
}

std::vector<Hex> FlowField::findPath(Hex start, Hex end) {
    if (!map.isWalkable(start) || !map.isWalkable(end)) return {};
    
    // 1. Check size and resize if needed (Handling Map Resize)
    size_t sz = (size_t)map.width * (size_t)map.height;
    if (parentMap.size() != sz) {
        parentMap.clear();
        parentMap.resize(sz, -1);
        built = false; // Force rebuild
    }

    // 2. Helper Lambda
    auto getIdx = [&](const Hex& h) {
        int col = h.q + (h.r - (h.r & 1)) / 2;
        int row = h.r;
        if (col < 0 || col >= map.width || row < 0 || row >= map.height) return -1;
        return row * map.width + col;
    };

    // 3. Initialize or Reset if target changed
    if (!built || end != currentEnd) {
        std::fill(parentMap.begin(), parentMap.end(), -1);
        currentEnd = end;
        
        std::queue<int> empty;
        std::swap(openQueue, empty);
        
        int endIdx = getIdx(end);
        if (endIdx != -1) {
            parentMap[endIdx] = endIdx; // Sentinel (points to self)
            openQueue.push(endIdx);
        }
        built = true; // "Built" now means initialized for this End
    }
    
    // 4. Incremental Build (Optimization)
    // Run BFS until start is reached OR queue is empty
    int startIdx = getIdx(start);
    if (startIdx == -1) return {};

    // Only run BFS if start is not yet visited
    if (parentMap[startIdx] == -1) {
        int steps = 0;
        // Run until we find start or exhaust queue
        // Optimization: limit steps per frame? No, we need path NOW.
        // Just run until found.
        while(!openQueue.empty()) {
            if (parentMap[startIdx] != -1) break; // Found start!

            int currIdx = openQueue.front();
            openQueue.pop();
            
            // Reconstruct Hex
            int r = currIdx / map.width;
            int c = currIdx % map.width;
            int q_ax = c - (r - (r & 1)) / 2;
            Hex currHex(q_ax, r);
            
            for (const auto& dir : HEX_DIRS) {
                Hex next = currHex + dir;
                
                // Fast boundary check via getIdx
                // Note: getIdx does boundary check, but map.isWalkable also checks grid content
                if (!map.isWalkable(next)) continue;
                
                int nextIdx = getIdx(next);
                if (nextIdx != -1 && parentMap[nextIdx] == -1) {
                    parentMap[nextIdx] = currIdx; // Point towards curr (which is closer to end)
                    openQueue.push(nextIdx);
                }
            }
            
            steps++;
        }
    }
    
    // 5. Trace Path
    if (parentMap[startIdx] == -1) return {}; // Unreachable

    std::vector<Hex> path;
    path.reserve(1000); // Pre-alloc
    
    VIZ_START(); 
    
    int currIdx = startIdx;
    int safety = 0;
    // Increased safety limit for large maps
    int maxSafety = map.width * map.height; 

    while (safety++ < maxSafety) {
        int r = currIdx / map.width;
        int c = currIdx % map.width;
        int q_ax = c - (r - (r & 1)) / 2;
        Hex h(q_ax, r);
        
        path.push_back(h);
        
        if (h == end) break;
        
        int nextIdx = parentMap[currIdx];
        
        // Visualization
        Hex nextHex = h;
        if (nextIdx != -1) {
            int nr = nextIdx / map.width;
            int nc = nextIdx % map.width;
            int nq = nc - (nr - (nr & 1)) / 2;
            nextHex = Hex(nq, nr);
        }
        VIZ_LOG(h);

        if (nextIdx == currIdx || nextIdx == -1) break; 
        currIdx = nextIdx;
    }
    
    return path;
}
