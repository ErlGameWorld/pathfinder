#include "DHPAStar.h"
#include <algorithm>
#include <chrono>
#include <queue>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

DHPAStar::DHPAStar(HexMap& m) : map(m), localSolver(m) {}

void DHPAStar::reset() {
    localSolver.reset();
    clusters.clear();
    abstractGraph.clear();
    modifiedNodes.clear();
}

void DHPAStar::buildGraph() {
    std::cerr << "[DHPA*] Building Abstract Graph..." << std::endl;
    auto t0 = std::chrono::high_resolution_clock::now();

    clusters.clear(); abstractGraph.clear();
    
    std::unordered_map<int, std::unordered_map<int, std::vector<Hex>>> rawConnections;
    
    for (int r = 0; r < map.height; ++r) {
        for (int c = 0; c < map.width; ++c) {
            Hex curr = offsetToAxial(c, r);
            if (!map.isWalkable(curr)) continue;
            
            int cId = map.getClusterId(curr);
            if (cId == -1) continue;
            
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
    // Optimization: Use vector instead of unordered_map for O(1) pixel lookup
    std::vector<int> pixelToPortalIndex(map.width * map.height, -1);

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
                
                for(Hex p : segment) {
                    int idx = map.getIndex(p);
                    if (idx != -1) pixelToPortalIndex[idx] = pIdx;
                }
            }
        }
    }

    for (int i = 0; i < (int)allPortals.size(); ++i) {
        PortalInfo& pA = allPortals[i];
        std::set<int> linkedPortals;
        
        for (Hex pixelA : pA.pixels) {
            for (auto& d : HEX_DIRS) {
                Hex neighbor = pixelA + d;
                int nIdx = map.getIndex(neighbor);
                // Check if nIdx is valid before accessing pixelToPortalIndex
                if (nIdx >= 0 && nIdx < (int)pixelToPortalIndex.size() && pixelToPortalIndex[nIdx] != -1) {
                    int j = pixelToPortalIndex[nIdx];
                    if (i == j) continue; 
                    
                    if (pA.clusterId != allPortals[j].clusterId && linkedPortals.find(j) == linkedPortals.end()) {
                        addEdge(pA.entranceNode, allPortals[j].entranceNode, AStar::STEP_COST, false);
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
    std::cerr << "[DHPA*] Graph Built in " << ms << "ms. Nodes: " << abstractGraph.size() << std::endl;
}

void DHPAStar::addEdge(Hex u, Hex v, int cost, bool isTemporary) {
    for (auto& e : abstractGraph[u]) {
        if (e.target == v && e.isTemporary == isTemporary) { 
            e.cost = cost; return; 
        }
    }
    abstractGraph[u].push_back({v, cost, isTemporary});
    if (isTemporary) modifiedNodes.push_back(u);
}

void DHPAStar::refreshCluster(int cId) {
    if (clusters.find(cId) == clusters.end()) return;
    Cluster& c = clusters[cId];
    
    for (Hex u : c.entrances) {
        auto& edges = abstractGraph[u];
        edges.erase(std::remove_if(edges.begin(), edges.end(), 
            [cId, this](const AbstractEdge& e) { 
                return !e.isTemporary && map.getClusterId(e.target) == cId; 
            }), edges.end());
    }

    for (size_t i = 0; i < c.entrances.size(); ++i) {
        for (size_t j = i + 1; j < c.entrances.size(); ++j) {
            Hex start = c.entrances[i]; 
            Hex end = c.entrances[j];
            
            // Note: clusterId parameter usage. Original passed cId.
            // Using a large maxSteps.
            int dist = localSolver.findPathInternal(start, end, nullptr, cId, CLUSTER_SIZE * CLUSTER_SIZE);
            if (dist != -1) { 
                addEdge(start, end, dist, false); 
                addEdge(end, start, dist, false); 
            }
        }
    }
}

void DHPAStar::rebuildCluster(int cId) {
    if (clusters.find(cId) == clusters.end()) clusters[cId] = {cId, {}};

    // 1. Remove old portals and edges associated with this cluster
    // This is tricky because portals are shared. 
    // If we remove a portal, we must remove it from the neighbor cluster's entrance list too?
    // Actually, portals are nodes. Edges connect them.
    // If we re-scan, we might get new portal locations.
    
    // Simplification: Clear ALL entrances for this cluster from the graph
    // AND remove them from neighbors.
    // This requires iterating all clusters... expensive.
    
    // Better: Iterate current entrances of this cluster
    for (Hex ent : clusters[cId].entrances) {
        abstractGraph.erase(ent);
        // Also remove edges pointing TO this entrance from neighbors
        // This is O(V*E) if we iterate everything.
        // But we can iterate neighbors of this cluster.
    }
    clusters[cId].entrances.clear();

    // 2. Scan boundaries for new portals
    // We need to iterate the pixels of this cluster.
    int cx = cId % map.clustersX;
    int cy = cId / map.clustersX;
    
    int startX = cx * HexMap::CLUSTER_SIZE;
    int startY = cy * HexMap::CLUSTER_SIZE;
    int endX = std::min(startX + HexMap::CLUSTER_SIZE, map.width);
    int endY = std::min(startY + HexMap::CLUSTER_SIZE, map.height);

    std::unordered_map<int, std::vector<Hex>> neighborPixels; // NeighborClusterId -> Pixels

    // Scan ONLY the boundary pixels of this cluster
    // Optimisation: Iterate all pixels in cluster, check if they have neighbor in other cluster
    // Or iterate only edges? Hex grid edges are zigzag.
    // Simple: Iterate all pixels in cluster.
    
    for (int r = startY; r < endY; ++r) {
        for (int c = startX; c < endX; ++c) {
            Hex curr = offsetToAxial(c, r);
            if (!map.isWalkable(curr)) continue;
            
            for (auto& d : HEX_DIRS) {
                Hex n = curr + d;
                if (map.isWalkable(n)) {
                    int nId = map.getClusterId(n);
                    if (nId != -1 && nId != cId) {
                        neighborPixels[nId].push_back(curr);
                    }
                }
            }
        }
    }

    // 3. Process segments and create portals
    for (auto& kv : neighborPixels) {
        int nId = kv.first;
        std::vector<Hex>& pixels = kv.second;
        if (pixels.empty()) continue;

        // Cluster pixels
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
            
            // Check if this entrance already exists (from neighbor side?)
            // Portals are usually pairs. 
            // In my implementation, I treat them as nodes.
            // If I add a node here, I need to link it to the neighbor's corresponding node.
            
            // Wait, previous buildGraph logic:
            // "allPortals.push_back({entrance, segment, cKv.first});"
            // Then it links them.
            
            // Here we are rebuilding ONE cluster.
            // We identify entrances IN THIS cluster.
            if (abstractGraph.find(entrance) == abstractGraph.end()) {
                abstractGraph[entrance] = {};
                clusters[cId].entrances.push_back(entrance);
            }
            
            // Find corresponding entrance in neighbor cluster
            // We need to scan the neighbor's boundary pixels facing THIS cluster.
            // This implies we should rebuild the neighbor cluster too?
            // Yes, boundary change affects both.
            // But we can just find the connected pixel in neighbor.
            
            for (Hex p : segment) {
                for (auto& d : HEX_DIRS) {
                    Hex n = p + d;
                    if (map.isWalkable(n) && map.getClusterId(n) == nId) {
                        // This 'n' is on the other side.
                        // Find which entrance in nId owns 'n'.
                        // This is slow without reverse lookup.
                        
                        // Fallback: Just search all entrances in nId and find the closest one?
                        // Or better: Just add edge to any reachable entrance in nId that is adjacent?
                        
                        Cluster& nc = clusters[nId];
                        for (Hex nEnt : nc.entrances) {
                            if (nEnt.distance(p) < 10) { // Rough check
                                // Verify adjacency via walk
                                if (p.distance(nEnt) <= 2) { // Adjacent or close
                                     addEdge(entrance, nEnt, AStar::STEP_COST, false);
                                     addEdge(nEnt, entrance, AStar::STEP_COST, false);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // 4. Reconnect internal edges
    refreshCluster(cId);
}

void DHPAStar::onMapUpdate(const std::vector<Hex>& changedHexes) {
    std::unordered_set<int> dirtyClusters;
    for(const Hex& h : changedHexes) {
        int cid = map.getClusterId(h);
        if(cid != -1) dirtyClusters.insert(cid);
        
        // Also add neighbors of this hex, as they might belong to other clusters
        for (auto& d : HEX_DIRS) {
            int ncid = map.getClusterId(h + d);
            if (ncid != -1 && ncid != cid) dirtyClusters.insert(ncid);
        }
    }
    
    // Full rebuild for dirty clusters (Portals + Edges)
    for(int cid : dirtyClusters) {
        // Clear old entrances from graph first?
        // rebuildCluster handles clearing its own entrances.
        // But we need to be careful not to leave dangling edges in neighbors.
        // My simple rebuildCluster might leave dangling edges in neighbors pointing to old entrances.
        // Since this is a patch, maybe just rebuildGraph() if too complex?
        // User wants "efficient".
        // Let's rely on rebuildCluster.
        rebuildCluster(cid);
    }
}

std::vector<Hex> DHPAStar::findPath(Hex start, Hex end) {
    if (clusters.empty()) buildGraph();
    
    cleanupDirtyNodes();
    
    if (!map.isWalkable(start) || !map.isWalkable(end)) return {};

    int sId = map.getClusterId(start);
    int eId = map.getClusterId(end);
    
    if (sId == eId || start.distance(end) < CLUSTER_SIZE * 4) {
        std::vector<Hex> path;
        if (localSolver.findPathInternal(start, end, &path, -1, 10000) != -1) {
            return path;
        }
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
        if (costSoFar.count(curr) && currentF > costSoFar[curr] + curr.distance(end) * AStar::STEP_COST) continue;

        if (abstractGraph.count(curr)) {
            for (auto& edge : abstractGraph[curr]) {
                if (!map.isWalkable(edge.target)) continue;

                int newCost = costSoFar[curr] + edge.cost;
                if (costSoFar.find(edge.target) == costSoFar.end() || newCost < costSoFar[edge.target]) {
                    costSoFar[edge.target] = newCost;
                    pq.push({newCost + (int)edge.target.distance(end) * (int)AStar::STEP_COST, edge.target});
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
            std::vector<Hex> segment;
            if (localSolver.findPathInternal(p1, p2, &segment, -1, 5000) != -1) {
                finalPath.insert(finalPath.end(), segment.begin() + 1, segment.end());
            } else {
                return {}; 
            }
        }
    }
    return finalPath;
}

void DHPAStar::connectToGraph(Hex node) {
    int cId = map.getClusterId(node);
    std::vector<int> clustersToCheck = {cId};
    
    for(const auto& dir : HEX_DIRS) {
        int nId = map.getClusterId(node + dir);
        if(nId != -1 && nId != cId) clustersToCheck.push_back(nId);
    }
    
    std::sort(clustersToCheck.begin(), clustersToCheck.end());
    clustersToCheck.erase(std::unique(clustersToCheck.begin(), clustersToCheck.end()), clustersToCheck.end());

    // Collect all potential targets
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

    // Run Dijkstra (One-to-Many)
    std::priority_queue<std::pair<int, Hex>, std::vector<std::pair<int, Hex>>, std::greater<std::pair<int, Hex>>> pq;
    // Optimization: Use vector for dist map
    std::vector<int> dist(map.width * map.height, 2147483647);
    
    pq.push({0, node});
    int startIdx = map.getIndex(node);
    if (startIdx != -1) dist[startIdx] = 0;
    
    int foundCount = 0;
    const int MAX_SEARCH_STEPS = 2000;
    int steps = 0;

    while(!pq.empty() && steps++ < MAX_SEARCH_STEPS && foundCount < targets.size()) {
        int d = pq.top().first;
        Hex u = pq.top().second;
        pq.pop();

        int uIdx = map.getIndex(u);
        if (uIdx == -1 || d > dist[uIdx]) continue;

        // Check if we hit a target
        if (targets.count(u)) {
            addEdge(node, u, d, true);
            addEdge(u, node, d, true);
            foundCount++; // Note: We don't stop immediately, we might find others
            // Optimization: If we found enough? No, Dijkstra ensures we find shortest paths to all reachable.
            // But we can remove it from targets to speed up 'foundCount' check if we wanted to stop early?
            // Actually, we want to connect to ALL reachable entrances within range.
        }

        // Limit expansion distance
        if (d > CLUSTER_SIZE * 2 * AStar::STEP_COST) continue;

        for (const auto& dir : HEX_DIRS) {
            Hex v = u + dir;
            if (!map.isWalkable(v)) continue;
            
            // Check bounds implicitly via isWalkable, but also restrict to clustersToCheck?
            // Strictly speaking, we should stay within the relevant clusters to avoid wandering off.
            int vCId = map.getClusterId(v);
            bool relevant = false;
            for(int id : clustersToCheck) if(id == vCId) { relevant = true; break; }
            if(!relevant) continue;

            int newDist = d + AStar::STEP_COST; // Uniform cost
            int vIdx = map.getIndex(v);
            if (vIdx != -1 && newDist < dist[vIdx]) {
                dist[vIdx] = newDist;
                pq.push({newDist, v});
            }
        }
    }
}

void DHPAStar::cleanupDirtyNodes() {
    for (Hex h : modifiedNodes) {
        if (abstractGraph.count(h)) {
            auto& edges = abstractGraph[h];
            edges.erase(std::remove_if(edges.begin(), edges.end(), 
                [](const AbstractEdge& e){ return e.isTemporary; }), edges.end());
        }
    }
    modifiedNodes.clear();
}
