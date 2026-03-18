#include "DHPAJps.h"
#include <algorithm>
#include <chrono>
#include <queue>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

DHPAJps::DHPAJps(HexMap& m) : map(m), localSolver(m), builderSolver(m) {}

void DHPAJps::reset() {
    localSolver.reset();
    builderSolver.reset();
    clusters.clear();
    abstractGraph.clear();
    modifiedNodes.clear();
}

void DHPAJps::buildGraph() {
    std::cerr << "[DHPA-JPS] Building Abstract Graph..." << std::endl;
    auto t0 = std::chrono::high_resolution_clock::now();

    clusters.clear(); abstractGraph.clear();
    
    // 1. Identify raw connections between clusters
    std::unordered_map<int, std::unordered_map<int, std::vector<Hex>>> rawConnections;
    
    for (int y = 0; y < map.height; ++y) {
        for (int x = 0; x < map.width; ++x) {
            Hex curr(x, y); // Axial from Offset? No, x,y are Offset loop vars usually in this project?
            // Wait, Hex(x,y) constructor takes q,r.
            // DHPAStar.cpp loop: for y..height, for x..width, Hex curr(x,y).
            // This implies x=q, y=r. 
            // BUT HexMap::getClusterId expects offset coords.
            // If the loop iterates q,r, we need to be careful.
            // HexMap stores grid in row-major offset order.
            // Let's check DHPAStar.cpp again.
            // It uses Hex curr(x,y). If map is 2500x2500, q,r can be large.
            // If map is defined as WxH, usually we iterate offset coords.
            // But Hex(x,y) constructs Axial.
            // The map.isWalkable(curr) handles axial.
            // So iterating q,r in bounding box is "okay" but might miss some if the map shape is skewed?
            // HexMap::HexMap(w,h) implies offset grid.
            // Correct iteration for offset grid:
            // for r=0..h, for c=0..w: Hex h = offsetToAxial(c,r).
            
            // Let's use the correct iteration for Offset Grid to ensure coverage.
            // Note: DHPAStar.cpp used Hex(x,y) directly. 
            // If HexMap is rectangular in Offset coords, iterating q,r (Axial) from 0..W, 0..H 
            // covers a parallelogram, NOT the rectangle!
            // This might be a bug in DHPAStar.cpp if W,H are offset dimensions.
            // Let's fix this here and in DHPAStar later.
            
            // Wait, Hex(q,r) constructor.
            // If the map is stored as Offset, iterating 0..W, 0..H in Axial 
            // will access valid indices BUT might miss half the map or go out of bounds?
            // map.isWalkable(Hex) does bounds check.
            
            // FIX: Iterate Offset Coordinates
             // Handled inside the loop below
        }
    }
    
    // Correct Loop:
    for (int r = 0; r < map.height; ++r) {
        for (int c = 0; c < map.width; ++c) {
             Hex curr = offsetToAxial(c, r);
             if (!map.isWalkable(curr)) continue;
             
             int cId = map.getClusterId(curr);
             if (cId == -1) continue; // Should not happen if in bounds
             
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
                if (nIdx >= 0 && nIdx < (int)pixelToPortalIndex.size() && pixelToPortalIndex[nIdx] != -1) {
                    int j = pixelToPortalIndex[nIdx];
                    if (i == j) continue; 
                    
                    if (pA.clusterId != allPortals[j].clusterId && linkedPortals.find(j) == linkedPortals.end()) {
                        // Inter-cluster edges are always cost 1 (adjacent pixels) * STEP_COST
                        // Or accurately: distance between entrance nodes?
                        // HPA* usually puts edge between entrance nodes.
                        // Distance is entranceA -> pixelA -> pixelB -> entranceB.
                        // For simplicity and speed, we approximate or calculate?
                        // Here we use AStar::STEP_COST (10) as base cost for transition.
                        // But accurate distance is better.
                        // Since they are adjacent clusters, dist is small.
                        int dist = pA.entranceNode.distance(allPortals[j].entranceNode) * 10;
                        addEdge(pA.entranceNode, allPortals[j].entranceNode, dist, false);
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
    std::cerr << "[DHPA-JPS] Graph Built in " << ms << "ms. Nodes: " << abstractGraph.size() << std::endl;
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
    
    // Remove old internal edges
    for (Hex u : c.entrances) {
        auto& edges = abstractGraph[u];
        edges.erase(std::remove_if(edges.begin(), edges.end(), 
            [cId, this](const AbstractEdge& e) { 
                return !e.isTemporary && map.getClusterId(e.target) == cId; 
            }), edges.end());
    }

    // Connect all pairs of entrances in this cluster
    // USE builderSolver (AStar) for constrained search!
    for (size_t i = 0; i < c.entrances.size(); ++i) {
        for (size_t j = i + 1; j < c.entrances.size(); ++j) {
            Hex start = c.entrances[i]; 
            Hex end = c.entrances[j];
            
            // Constrained search within cluster
            int dist = builderSolver.findPathInternal(start, end, nullptr, cId, HexMap::CLUSTER_SIZE * HexMap::CLUSTER_SIZE * 2);
            if (dist != -1) { 
                addEdge(start, end, dist, false); 
                addEdge(end, start, dist, false); 
            }
        }
    }
}

void DHPAJps::rebuildCluster(int cId) {
    // Same logic as DHPAStar::rebuildCluster
    if (clusters.find(cId) == clusters.end()) clusters[cId] = {cId, {}};

    for (Hex ent : clusters[cId].entrances) {
        abstractGraph.erase(ent);
    }
    clusters[cId].entrances.clear();

    int cx = cId % map.clustersX;
    int cy = cId / map.clustersX;
    
    int startX = cx * HexMap::CLUSTER_SIZE;
    int startY = cy * HexMap::CLUSTER_SIZE;
    int endX = std::min(startX + HexMap::CLUSTER_SIZE, map.width);
    int endY = std::min(startY + HexMap::CLUSTER_SIZE, map.height);

    std::unordered_map<int, std::vector<Hex>> neighborPixels; 

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

    for (auto& kv : neighborPixels) {
        int nId = kv.first;
        std::vector<Hex>& pixels = kv.second;
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
                clusters[cId].entrances.push_back(entrance);
            }
            
            for (Hex p : segment) {
                for (auto& d : HEX_DIRS) {
                    Hex n = p + d;
                    if (map.isWalkable(n) && map.getClusterId(n) == nId) {
                        Cluster& nc = clusters[nId];
                        for (Hex nEnt : nc.entrances) {
                            if (nEnt.distance(p) < 10) { 
                                if (p.distance(nEnt) <= 2) { 
                                     addEdge(entrance, nEnt, 10, false);
                                     addEdge(nEnt, entrance, 10, false);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    refreshCluster(cId);
}

void DHPAJps::onMapUpdate(const std::vector<Hex>& changedHexes) {
    std::unordered_set<int> dirtyClusters;
    for(const Hex& h : changedHexes) {
        int cid = map.getClusterId(h);
        if(cid != -1) dirtyClusters.insert(cid);
        for (auto& d : HEX_DIRS) {
            int ncid = map.getClusterId(h + d);
            if (ncid != -1 && ncid != cid) dirtyClusters.insert(ncid);
        }
    }
    
    localSolver.reset(); 
    
    for(int cid : dirtyClusters) {
        rebuildCluster(cid);
    }
}

void DHPAJps::connectToGraph(Hex node) {
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
                if (map.isWalkable(ent) && node.distance(ent) < HexMap::CLUSTER_SIZE * 2) {
                    targets.insert(ent);
                }
            }
        }
    }

    if (targets.empty()) return;

    // Use Dijkstra for connection
    std::priority_queue<std::pair<int, Hex>, std::vector<std::pair<int, Hex>>, std::greater<std::pair<int, Hex>>> pq;
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
        
        if (targets.count(u)) {
            // Found a target
            addEdge(node, u, d, true);
            addEdge(u, node, d, true); // Bi-directional temporary connection
            foundCount++;
        }

        for (const auto& dir : HEX_DIRS) {
            Hex v = u + dir;
            if (!map.isWalkable(v)) continue;
            
            int vIdx = map.getIndex(v);
            if (vIdx == -1) continue;
            
            int newDist = d + 10; // Cost 10
            if (newDist < dist[vIdx]) {
                dist[vIdx] = newDist;
                pq.push({newDist, v});
            }
        }
    }
}

void DHPAJps::cleanupDirtyNodes() {
    for (Hex u : modifiedNodes) {
        auto& edges = abstractGraph[u];
        edges.erase(std::remove_if(edges.begin(), edges.end(), 
            [](const AbstractEdge& e) { return e.isTemporary; }), edges.end());
    }
    modifiedNodes.clear();
    // Also remove the start/end nodes from abstractGraph if they were added purely temporarily
    // But since we use unordered_map, empty vector is fine.
}

std::vector<Hex> DHPAJps::findPath(Hex start, Hex end) {
    if (clusters.empty()) buildGraph();
    
    cleanupDirtyNodes();
    
    if (!map.isWalkable(start) || !map.isWalkable(end)) return {};

    int sId = map.getClusterId(start);
    int eId = map.getClusterId(end);
    
    // Direct search if close
    if (sId == eId || start.distance(end) < HexMap::CLUSTER_SIZE * 4) {
        // Use JPS for direct search!
        return localSolver.findPath(start, end);
    }

    if (abstractGraph.find(start) == abstractGraph.end()) abstractGraph[start] = {};
    if (abstractGraph.find(end) == abstractGraph.end()) abstractGraph[end] = {};

    connectToGraph(start);
    connectToGraph(end);

    // Abstract Search (A*)
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
        if (costSoFar.count(curr) && currentF > costSoFar[curr] + curr.distance(end) * 10) continue;

        if (abstractGraph.count(curr)) {
            for (auto& edge : abstractGraph[curr]) {
                // Check walkability of target (might be dynamic obstacle)
                if (!map.isWalkable(edge.target)) continue;

                int newCost = costSoFar[curr] + edge.cost;
                if (costSoFar.find(edge.target) == costSoFar.end() || newCost < costSoFar[edge.target]) {
                    costSoFar[edge.target] = newCost;
                    pq.push({newCost + (int)edge.target.distance(end) * 10, edge.target});
                    cameFrom[edge.target] = curr;
                }
            }
        }
    }

    if (!found) return {};

    // Reconstruct Abstract Path
    std::vector<Hex> abstractPath;
    Hex curr = end;
    while (curr != start) { 
        abstractPath.push_back(curr); 
        curr = cameFrom[curr]; 
    }
    abstractPath.push_back(start);
    std::reverse(abstractPath.begin(), abstractPath.end());

    // Refine Path using JPS
    std::vector<Hex> finalPath;
    finalPath.push_back(start);

    for (size_t i = 0; i < abstractPath.size() - 1; ++i) {
        Hex p1 = abstractPath[i];
        Hex p2 = abstractPath[i+1];
        
        if (p1 == p2) continue;

        if (p1.distance(p2) <= 1) {
            finalPath.push_back(p2);
        } else {
            // Use JPS for refinement
            // Note: JPS::findPath returns full path including start.
            std::vector<Hex> segment = localSolver.findPath(p1, p2);
            if (!segment.empty()) {
                // Skip the first element (p1) as it's already in finalPath
                finalPath.insert(finalPath.end(), segment.begin() + 1, segment.end());
            } else {
                // Fallback? If abstract edge exists, path should exist.
                // Unless JPS fails where A* succeeded (unlikely if map static).
                // Or obstacle appeared.
                return {}; 
            }
        }
    }
    return finalPath;
}
