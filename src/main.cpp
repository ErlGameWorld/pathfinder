#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <chrono>
#include <iomanip>
#include <unordered_map>
#include <memory>
#include <algorithm>

#include "common/Types.h"
#include "common/Hex.h"
#include "common/VizMacros.h"
#include "map/HexMap.h"
#include "map/MapGenerator.h"
#include "finder/IFinder.h"
#include "finder/DHPAStar.h"
#include "finder/AStar.h"
#include "finder/BFS.h"
#include "finder/BiAStar.h"
#include "finder/JPS.h"
#include "finder/FlowField.h"
#include "finder/DHPAJps.h"
#include "finder/HJPS.h"


#ifdef VIZ_SUPPORT
    std::vector<Hex> g_vizVisitedLog;
    bool g_vizEnabled = false;
#endif

// Finder Factory
std::unordered_map<std::string, std::unique_ptr<IFinder>> g_finders;

void registerFinders(HexMap& map) {
    g_finders["AStar"] = std::make_unique<AStar>(map);
    g_finders["JPS"] = std::make_unique<JPS>(map);
    g_finders["HJPS"] = std::make_unique<HJPS>(map);
    g_finders["BFS"] = std::make_unique<BFS>(map);
    g_finders["BiAStar"] = std::make_unique<BiAStar>(map);
    g_finders["DHPAStar"] = std::make_unique<DHPAStar>(map);
    g_finders["DHPAJps"] = std::make_unique<DHPAJps>(map);
    g_finders["FlowField"] = std::make_unique<FlowField>(map);
}

struct BenchmarkResult {
    std::string name;
    size_t pathLen;
    double avgTimeUs;
    bool foundPath;
};

void runBenchmark(Hex start, Hex end, int iterations, std::string label) {
    std::cout << "------------------------------------------------" << std::endl;
    std::cout << "Benchmark: " << label << " (" << start.q << "," << start.r << ") -> (" << end.q << "," << end.r << ")" << std::endl;
    
    std::vector<BenchmarkResult> results;
    results.reserve(g_finders.size());
    
    // 先收集所有结果
    for (auto& kv : g_finders) {
        IFinder* finder = kv.second.get();
        
        auto path = finder->findPath(start, end);
        if (path.empty()) {
            results.push_back({finder->getName(), 0, 0.0, false});
            continue;
        }
        
        auto t_start = std::chrono::high_resolution_clock::now();
        size_t totalPathLen = 0;
        for (int i = 0; i < iterations; ++i) {
            auto p = finder->findPath(start, end);
            totalPathLen += p.size();
        }
        auto t_end = std::chrono::high_resolution_clock::now();
        (void)totalPathLen;
        
        double total_us = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
        double avg_us = total_us / iterations;
        
        results.push_back({finder->getName(), path.size(), avg_us, true});
    }
    
    // 按耗时排序（快的在前）
    std::sort(results.begin(), results.end(), [](const BenchmarkResult& a, const BenchmarkResult& b) {
        // 找到路径的排前面，再按时间排序
        if (a.foundPath != b.foundPath) return a.foundPath > b.foundPath;
        return a.avgTimeUs < b.avgTimeUs;
    });
    
    // 按排序后的顺序输出
    for (const auto& r : results) {
        std::cout << "  Algorithm: " << r.name << std::endl;
        if (!r.foundPath) {
            std::cout << "    [Result] No path found!" << std::endl;
            continue;
        }
        std::cout << "    [Result] Path length: " << r.pathLen << std::endl;
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "    [Perf] Avg Time: " << r.avgTimeUs << " us (" << r.avgTimeUs / 1000.0 << " ms)" << std::endl;
    }
}

// JSON helper (optimized string concatenation)
std::string formatBenchmarkResult(const std::string& name, size_t pathLen, double timeUs, size_t visitedCount) {
    std::string result;
    result.reserve(128 + name.size());
    result += "{\"name\":\"";
    result += name;
    result += "\",\"pathLength\":";
    result += std::to_string(pathLen);
    result += ",\"timeUs\":";
    
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.2f", timeUs);
    result += buf;
    
    result += ",\"visited\":";
    result += std::to_string(visitedCount);
    result += "}";
    return result;
}

struct InteractiveResult {
    std::string name;
    size_t pathLen;
    double timeUs;
};

void runBenchmarkInteractive(Hex start, Hex end, const std::vector<std::string>& filter = {}) {
    std::vector<InteractiveResult> results;
    results.reserve(g_finders.size());
    
    // 先收集所有结果
    for (auto& kv : g_finders) {
        if (!filter.empty()) {
            bool found = false;
            for (const auto& name : filter) {
                if (name == kv.first) { found = true; break; }
            }
            if (!found) continue;
        }

        IFinder* finder = kv.second.get();
        
        // Warmup
        finder->findPath(start, end);
        
        // Measure with result accumulation to prevent optimization
        auto t_start = std::chrono::high_resolution_clock::now();
        const int iterations = 10;
        size_t totalPathLen = 0;
        std::vector<Hex> lastPath;
        for (int i = 0; i < iterations; ++i) {
            lastPath = finder->findPath(start, end);
            totalPathLen += lastPath.size();
        }
        auto t_end = std::chrono::high_resolution_clock::now();
        (void)totalPathLen;
        
        double total_us = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
        double avg_us = total_us / iterations;
        
        results.push_back({finder->getName(), lastPath.size(), avg_us});
    }
    
    // 按耗时排序（快的在前）
    std::sort(results.begin(), results.end(), [](const InteractiveResult& a, const InteractiveResult& b) {
        return a.timeUs < b.timeUs;
    });
    
    // 输出排序后的 JSON
    std::cout << "BENCH_START" << std::endl;
    std::cout << "["; // Start JSON array
    
    bool first = true;
    for (const auto& r : results) {
        if (!first) std::cout << ",";
        std::cout << formatBenchmarkResult(r.name, r.pathLen, r.timeUs, 0);
        first = false;
    }
    
    std::cout << "]" << std::endl; // End JSON array
    std::cout << "BENCH_END" << std::endl;
}

void runInteractive(HexMap& map) {
    // Protocol:
    // P col1 row1 col2 row2 algoName -> Find Path
    // B col1 row1 col2 row2          -> Run Benchmark (All Algos)
    // O col row state                -> Set Obstacle
    // G                              -> Generate Map
    // L ...                          -> Load Map
    // Q                              -> Quit
    
    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty()) continue;
        
        std::stringstream ss(line);
        char cmd;
        ss >> cmd;
        
        if (cmd == 'G') {
            int w = -1, h = -1;
            if (ss >> w >> h) {
                if (w > 0 && h > 0 && (w != map.width || h != map.height)) {
                     std::cerr << "[Interactive] Resizing map to " << w << "x" << h << std::endl;
                     map.resize(w, h);
                     for(auto& kv : g_finders) kv.second->reset();
                }
            }
            
            std::fill(map.grid.begin(), map.grid.end(), 0);

            MapGenerator generator(map.width, map.height);
            generator.generateAll();
            generator.applyToMap(map);
            
            // Rebuild Graphs
            std::cout << "[Interactive] Rebuilding graphs..." << std::endl;
            for(auto& kv : g_finders) kv.second->buildGraph();
            
            std::cout << "MAP_DATA " << generator.serializeRLE() << std::endl;
            
        } else if (cmd == 'L') {
            int w, h;
            if (ss >> w >> h) {
                if (w > 0 && h > 0) {
                    if (w != map.width || h != map.height) {
                        std::cerr << "[Interactive] Resizing map for Load to " << w << "x" << h << std::endl;
                        map.resize(w, h);
                        for(auto& kv : g_finders) kv.second->reset();
                    } else {
                        std::fill(map.grid.begin(), map.grid.end(), 0);
                    }
                    
                    int count, val;
                    int idx = 0;
                    int maxIdx = w * h;
                    
                    while (ss >> count >> val) {
                        uint8_t cellVal = (val > 0) ? 1 : 0;
                        int end = std::min(idx + count, maxIdx);
                        for (; idx < end; ++idx) {
                            map.grid[idx] = cellVal;
                        }
                    }
                    
                    std::cout << "[Interactive] Rebuilding graphs..." << std::endl;
                    for(auto& kv : g_finders) kv.second->buildGraph();
                    std::cout << "OK" << std::endl;
                }
            }
        } else if (cmd == 'P') {
            int c1, r1, c2, r2;
            std::string algoName;
            ss >> c1 >> r1 >> c2 >> r2 >> algoName;
            
            Hex start = offsetToAxial(c1, r1);
            Hex end = offsetToAxial(c2, r2);
            
            if (g_finders.find(algoName) == g_finders.end()) {
                // Default to DHPAStar if not found or empty
                if (!g_finders.empty()) algoName = g_finders.begin()->first;
                else {
                    std::cout << "NOPATH" << std::endl;
                    std::cout << "RESULT_END" << std::endl;
                    continue;
                }
            }
            
            VIZ_START();
            
            std::vector<Hex> path = g_finders[algoName]->findPath(start, end);
            
            std::cout << "RESULT_START" << std::endl;
            if (!path.empty()) {
                std::cout << "PATH " << path.size() << " ";
                for (auto& h : path) {
                    Hex off = axialToOffset(h.q, h.r);
                    std::cout << off.q << "," << off.r << " ";
                }
                std::cout << std::endl;
            } else {
                std::cout << "NOPATH" << std::endl;
            }

            VIZ_PRINT_RESULTS();
            VIZ_STOP();

            std::cout << "RESULT_END" << std::endl;
            
        } else if (cmd == 'B') {
            int c1, r1, c2, r2;
            ss >> c1 >> r1 >> c2 >> r2;
            
            std::vector<std::string> filter;
            std::string algo;
            while (ss >> algo) {
                if (!algo.empty()) filter.push_back(algo);
            }

            Hex start = offsetToAxial(c1, r1);
            Hex end = offsetToAxial(c2, r2);
            runBenchmarkInteractive(start, end, filter);
        } else if (cmd == 'O') {
            int c, r, state;
            ss >> c >> r >> state;
            
            map.setObstacleOffset(c, r, state != 0);
            Hex h = offsetToAxial(c, r);
            for(auto& kv : g_finders) kv.second->onMapUpdate({h});
            
            std::cout << "OK" << std::endl;
        } else if (cmd == 'C') {
            int removed = 0;
            for (auto& v : map.grid) {
                if (v) {
                    v = 0;
                    removed++;
                }
            }
            
            // 重置 finder 缓存
            for(auto& kv : g_finders) {
                kv.second->reset();
                kv.second->buildGraph();
            }
            
            std::cout << "CLEANED " << removed << std::endl;
        } else if (cmd == 'Q') {
            break;
        }
    }
}

int main(int argc, char** argv) {
    bool interactive = false;
    if (argc > 1 && std::string(argv[1]) == "--server") {
        interactive = true;
    }

    int W = 2500;
    int H = 2500;
    if (interactive) {
        W = 250;
        H = 250;
    }
    
    if (!interactive) {
        std::cout << "[Init] Allocating Map (" << W << "x" << H << ")..." << std::endl;
    }
    
    HexMap map(W, H);
    
    // Register algorithms
    registerFinders(map);
    
    if (!interactive) {
        MapGenerator generator(W, H);
        generator.generateAll();
        generator.applyToMap(map);
    }

    // Build initial graphs
    std::cout << "[Main] Building hierarchical graphs..." << std::endl;
    for(auto& kv : g_finders) {
        kv.second->buildGraph();
    }

    if (interactive) {
        std::cerr << "Server Ready" << std::endl; 
        runInteractive(map);
    } else {
        std::cout << "\n=== START BENCHMARKS ===\n" << std::endl;

        runBenchmark(Hex(200, 200), Hex(220, 220), 5000, "Short Distance");
        runBenchmark(Hex(500, 500), Hex(1000, 1000), 100, "Medium Distance");
        runBenchmark(Hex(100, 100), Hex(2300, 2300), 50, "Long Distance");

        std::cout << "\n=== DYNAMIC UPDATE TEST ===\n" << std::endl;
        Hex blockPoint(1250, 1250); 
        map.setObstacle(blockPoint, true);
        //for(auto& kv : g_finders) kv.second->onMapUpdate({blockPoint});
        
        runBenchmark(Hex(100, 100), Hex(2300, 2300), 50, "Long Distance (After Update)");
    }

    return 0;
}
