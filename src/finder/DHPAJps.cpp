#include "DHPAJps.h"
#include <algorithm>
#include <chrono>
#include <queue>
#include <iostream>
#include <map>
#include <unordered_set>

DHPAJps::DHPAJps(HexMap& m) : map(m), localSolver(m) {}

void DHPAJps::reset() {
    localSolver.reset();
    clusters.clear();
    abstractGraph.clear();
    modifiedNodes.clear();
}

void DHPAJps::buildGraph() {
    std::cerr << "[DHPAJps] Building Abstract Graph..." << std::endl;
    auto t0 = std::chrono::high_resolution_clock::now();

    clusters.clear(); abstractGraph.clear();
    
    // Use same logic as DHPAStar for initial portal detection
    // Logic is identical, only intra-cluster connection differs
    
    std::map<int, std::map<int, std::vector<Hex>>> rawConnections;
    
    for (int y = 0; y < map.height; ++y) {
        for (int x = 0; x < map.width; ++x) {
            Hex curr(x, y);
            if (!map.isWalkable(curr)) continue;
            
            int cId = map.getClusterId(curr);
            for (auto& d : HEX_DIRS) {
                Hex n = curr + d;
                if (map.isWalkable(n)) {
                    int nId = map.getClusterId(n);
                    if (nId != -1 && nId != cId) {
                        rawConnections[cId][nId].push_back(curr);
                    }
                }
            }
        }
    }

    std::vector<PortalInfo> allPortals;
    std::unordered_map<Hex, int> pixelToPortalIndex;

    for (auto& cKv : rawConnections) {
        for (auto& nKv : cKv.second) {
            std::vector<Hex>& pixels = nKv.second;
            if (pixels.empty()) continue;

            std::vector<bool> visited(pixels.size(), false);
            for(size_t i = 0; i < pixels.size(); ++i) {
                if(visited[i]) continue;
                
                std::vector<Hex> segment; 
                std::queue<Hex> q;
                q.push(pixels[i]); visited[i] = true; segment.push_back(pixels[i]);
                
                size_t head = 0;
                while(head < segment.size()) {
                    Hex h = segment[head++];
                    for(size_t j = i + 1; j < pixels.size(); ++j) {
                        if(!visited[j] && h.distance(pixels[j]) == 1) {
                            visited[j] = true; 
                            segment.push_back(pixels[j]);
                        }
                    }
                }

                Hex entrance = segment[segment.size() / 2];
                if (abstractGraph.find(entrance) == abstractGraph.end()) {
                    abstractGraph[entrance] = {};
                    clusters[cKv.first].entrances.push_back(entrance);
                }

                int pIdx = (int)allPortals.size();
                allPortals.push_back({entrance, segment, cKv.first});
                for(Hex p : segment) pixelToPortalIndex[p] = pIdx;
            }
        }
    }

    for (int i = 0; i < (int)allPortals.size(); ++i) {
        PortalInfo& pA = allPortals[i];
        std::set<int> linkedPortals;
        
        for (Hex pixelA : pA.pixels) {
            for (auto& d : HEX_DIRS) {
                Hex neighbor = pixelA + d;
                if (pixelToPortalIndex.count(neighbor)) {
                    int j = pixelToPortalIndex[neighbor];
                    if (i == j) continue; 
                    
                    if (pA.clusterId != allPortals[j].clusterId && linkedPortals.find(j) == linkedPortals.end()) {
                        addEdge(pA.entranceNode, allPortals[j].entranceNode, 1, false);
                        linkedPortals.insert(j);
                    }
                }
            }
        }
    }

    for (auto& kv : clusters) {
        refreshCluster(kv.first);
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    std::cerr << "[DHPAJps] Graph Built in " << ms << "ms. Nodes: " << abstractGraph.size() << std::endl;
}

void DHPAJps::addEdge(Hex u, Hex v, int cost, bool isTemporary) {
    for (auto& e : abstractGraph[u]) {
        if (e.target == v && e.isTemporary == isTemporary) { 
            e.cost = cost; return; 
        }
    }
    abstractGraph[u].push_back({v, cost, isTemporary});
    if (isTemporary) modifiedNodes.push_back(u);
}

void DHPAJps::refreshCluster(int cId) {
    if (clusters.find(cId) == clusters.end()) return;
    Cluster& c = clusters[cId];
    
    for (Hex u : c.entrances) {
        auto& edges = abstractGraph[u];
        edges.erase(std::remove_if(edges.begin(), edges.end(), 
            [cId, this](const AbstractEdge& e) { 
                return !e.isTemporary && map.getClusterId(e.target) == cId; 
            }), edges.end());
    }

    // Use JPS for intra-cluster connections?
    // JPS finds paths on the full grid.
    // However, for intra-cluster connections, we ideally only want paths that stay INSIDE the cluster.
    // JPS jumps might jump OUT of the cluster.
    // Standard DHPA* uses A* with clusterId constraint.
    // JPS doesn't natively support "stay in region" constraint easily without modifying JPS logic.
    // If we use standard JPS, it might go outside.
    // But for "DHPAJps", maybe the intent is just to use JPS for speed?
    // If the path goes outside, it's still a valid path in the map.
    // But DHPA* abstract graph assumes edges represent reachability.
    // If we strictly want to stay inside, JPS needs modification.
    // Given the user request "DHPAJps", let's use JPS freely. 
    // If it goes outside, the cost is real.
    
    // Performance Optimization: Limit JPS depth or complexity if too many entrances
    // O(N^2) JPS calls where N is entrances count.
    // In complex maps, N can be large (e.g., river passing through cluster).
    
    // 如果入口太多，只连接距离较近的，避免 O(N^2) 爆炸
    size_t limit = c.entrances.size();
    bool useHeuristicPruning = false;
    if (limit > 15) {
        useHeuristicPruning = true;
    }

    for (size_t i = 0; i < limit; ++i) {
        for (size_t j = i + 1; j < limit; ++j) {
            Hex start = c.entrances[i]; 
            Hex end = c.entrances[j];
            
            // 简单剪枝：如果两点距离太远（超过 Cluster 尺寸），则不尝试连接
            // 因为它们很可能不属于同一个凸区域，或者通过其他中间节点连接更好。
            if (useHeuristicPruning && start.distance(end) > CLUSTER_SIZE) {
                continue;
            }
            
            // Critical Optimization:
            // JPS reset() clears the ENTIRE grid state (nodeData).
            // Calling localSolver.findPath() inside this loop (which runs for every cluster)
            // means we are resetting a 500x500 grid thousands of times!
            // That is extremely slow and causes the crash/timeout.
            // 
            // Solution: 
            // 1. Don't use JPS for intra-cluster (short paths). Use A* or BFS which is lighter?
            //    But we are DHPAJps.
            // 2. Optimize JPS reset to only clear dirty nodes?
            //    Our JPS implementation uses visitedGen to avoid full clear.
            //    Let's check JPS::reset().
            //    It calls std::fill(nodeData...). That is O(Width*Height).
            //    In a loop of O(Clusters * Entrances^2), this is O(MapSize^2) roughly.
            //
            // FIX: We must NOT call reset() inside findPath() if we can avoid it, 
            // or we must make reset() cheaper.
            // JPS::findPath calls currentGen++ which is cheap.
            // BUT JPS::findPath() MIGHT call reset() if currentGen wraps around?
            // No, JPS::findPath calls reset() only if currentGen == 0.
            //
            // WAIT! does localSolver.findPath() call reset()?
            // No, usually it just increments gen.
            //
            // Let's look at the crash log. 3221225477 = 0xC0000005 (Access Violation).
            // It crashes during "Building Abstract Graph".
            // Likely memory corruption or stack overflow in JPS recursion?
            //
            // We just fixed JPS recursion depth.
            //
            // Maybe the issue is we are running JPS on a graph that is being built?
            // No, localSolver uses 'map'.
            //
            // Let's try to limit the number of JPS calls or make them safer.
            // Or maybe the vector path return is too heavy?
            
            std::vector<Hex> path = localSolver.findPath(start, end);
            if (!path.empty()) {
                addEdge(start, end, path.size(), false); 
                addEdge(end, start, path.size(), false); 
            }
        }
    }
}

void DHPAJps::onMapUpdate(const std::vector<Hex>& changedHexes) {
    std::unordered_set<int> dirtyClusters;
    for(const Hex& h : changedHexes) {
        int cid = map.getClusterId(h);
        if(cid != -1) dirtyClusters.insert(cid);
    }
    for(int cid : dirtyClusters) {
        refreshCluster(cid);
    }
}

std::vector<Hex> DHPAJps::findPath(Hex start, Hex end) {
    cleanupDirtyNodes();
    
    if (!map.isWalkable(start) || !map.isWalkable(end)) return {};

    int sId = map.getClusterId(start);
    int eId = map.getClusterId(end);
    
    // Quick JPS if close
    if (sId == eId || start.distance(end) < CLUSTER_SIZE * 1.5) {
        return localSolver.findPath(start, end);
    }

    if (abstractGraph.find(start) == abstractGraph.end()) abstractGraph[start] = {};
    if (abstractGraph.find(end) == abstractGraph.end()) abstractGraph[end] = {};

    connectToGraph(start);
    connectToGraph(end);

    std::unordered_map<Hex, Hex> cameFrom;
    std::unordered_map<Hex, int> costSoFar;
    typedef std::pair<int, Hex> PNode;
    std::priority_queue<PNode, std::vector<PNode>, std::greater<PNode>> pq;

    pq.push({0, start});
    cameFrom[start] = start;
    costSoFar[start] = 0;
    
    bool found = false;
    while (!pq.empty()) {
        Hex curr = pq.top().second;
        int currentF = pq.top().first;
        pq.pop();

        if (curr == end) { found = true; break; }
        if (costSoFar.count(curr) && currentF > costSoFar[curr] + curr.distance(end)) continue;

        if (abstractGraph.count(curr)) {
            for (auto& edge : abstractGraph[curr]) {
                if (!map.isWalkable(edge.target)) continue;

                int newCost = costSoFar[curr] + edge.cost;
                if (costSoFar.find(edge.target) == costSoFar.end() || newCost < costSoFar[edge.target]) {
                    costSoFar[edge.target] = newCost;
                    pq.push({newCost + edge.target.distance(end), edge.target});
                    cameFrom[edge.target] = curr;
                }
            }
        }
    }

    if (!found) return {};

    std::vector<Hex> abstractPath;
    Hex curr = end;
    while (curr != start) { 
        abstractPath.push_back(curr); 
        curr = cameFrom[curr]; 
    }
    abstractPath.push_back(start);
    std::reverse(abstractPath.begin(), abstractPath.end());

    std::vector<Hex> finalPath;
    finalPath.push_back(start);

    for (size_t i = 0; i < abstractPath.size() - 1; ++i) {
        Hex p1 = abstractPath[i];
        Hex p2 = abstractPath[i+1];
        
        if (p1 == p2) continue;

        if (p1.distance(p2) <= 1) {
            finalPath.push_back(p2);
        } else {
            // Use JPS for segment refinement
            std::vector<Hex> segment = localSolver.findPath(p1, p2);
            if (!segment.empty()) {
                finalPath.insert(finalPath.end(), segment.begin() + 1, segment.end());
            } else {
                return {}; 
            }
        }
    }
    return finalPath;
}

void DHPAJps::connectToGraph(Hex node) {
    // Same logic as DHPAStar - Dijkstra is optimal for finding all nearby entrances
    int cId = map.getClusterId(node);
    std::vector<int> clustersToCheck = {cId};
    
    for(const auto& dir : HEX_DIRS) {
        int nId = map.getClusterId(node + dir);
        if(nId != -1 && nId != cId) clustersToCheck.push_back(nId);
    }
    
    std::sort(clustersToCheck.begin(), clustersToCheck.end());
    clustersToCheck.erase(std::unique(clustersToCheck.begin(), clustersToCheck.end()), clustersToCheck.end());

    std::unordered_set<Hex> targets;
    for (int id : clustersToCheck) {
        if (clusters.count(id)) {
            for (Hex ent : clusters[id].entrances) {
                if (map.isWalkable(ent) && node.distance(ent) < CLUSTER_SIZE * 1.5) {
                    targets.insert(ent);
                }
            }
        }
    }

    if (targets.empty()) return;

    std::priority_queue<std::pair<int, Hex>, std::vector<std::pair<int, Hex>>, std::greater<std::pair<int, Hex>>> pq;
    std::unordered_map<Hex, int> dist;
    
    pq.push({0, node});
    dist[node] = 0;
    
    int foundCount = 0;
    const int MAX_SEARCH_STEPS = 2000;
    int steps = 0;

    while(!pq.empty() && steps++ < MAX_SEARCH_STEPS && foundCount < targets.size()) {
        int d = pq.top().first;
        Hex u = pq.top().second;
        pq.pop();

        if (d > dist[u]) continue;

        if (targets.count(u)) {
            addEdge(node, u, d, true);
            addEdge(u, node, d, true);
            foundCount++;
        }

        if (d > CLUSTER_SIZE * 2) continue;

        for (const auto& dir : HEX_DIRS) {
            Hex v = u + dir;
            if (!map.isWalkable(v)) continue;
            
            int vCId = map.getClusterId(v);
            bool relevant = false;
            for(int id : clustersToCheck) if(id == vCId) { relevant = true; break; }
            if(!relevant) continue;

            int newDist = d + 1;
            if (dist.find(v) == dist.end() || newDist < dist[v]) {
                dist[v] = newDist;
                pq.push({newDist, v});
            }
        }
    }
}

void DHPAJps::cleanupDirtyNodes() {
    for (Hex h : modifiedNodes) {
        if (abstractGraph.count(h)) {
            auto& edges = abstractGraph[h];
            edges.erase(std::remove_if(edges.begin(), edges.end(), 
                [](const AbstractEdge& e){ return e.isTemporary; }), edges.end());
        }
    }
    modifiedNodes.clear();
}
