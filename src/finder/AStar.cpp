#include "AStar.h"
#include "../common/VizMacros.h"
#include <algorithm>
#include <cmath>
#include <cstdlib> // std::llabs

AStar::AStar(const HexMap& m) : map(m) {
    size_t size = (size_t)map.width * (size_t)map.height;
    nodeData.resize(size);
    openSetContainer.reserve(size / 8);
}

void AStar::reset() {
    size_t requiredSize = (size_t)map.width * (size_t)map.height;

    if (nodeData.size() != requiredSize) {
        nodeData.resize(requiredSize);
        openSetContainer.reserve(requiredSize / 8);
        currentGen = 0;
    } else {
        currentGen = 0;
    }
}

std::vector<Hex> AStar::findPath(Hex start, Hex end) {
    std::vector<Hex> path;
    if (findPathInternal(start, end, &path) != -1) {
        return path;
    }
    return {};
}

int64_t AStar::findPathInternal(Hex start, Hex end, std::vector<Hex>* outPath, int clusterId, int maxSteps) {
    // ---------------------------------------------------------
    // 1. 预处理与安全检查
    // ---------------------------------------------------------

    int sCol, sRow, eCol, eRow;
    hexToOffset(start, sCol, sRow);
    hexToOffset(end, eCol, eRow);

    int startIdx = getMapIndex(sCol, sRow);
    int endIdx = getMapIndex(eCol, eRow);

    // 越界检查
    if (startIdx == -1 || endIdx == -1) return -1;

    // 阻挡检查
    if (!map.isWalkable(start) || !map.isWalkable(end)) return -1;

    // Cluster 剪枝
    if (clusterId != -1) {
        if (map.getClusterId(start) != clusterId || map.getClusterId(end) != clusterId) {
            return -1;
        }
    }

    // 起点即终点
    if (startIdx == endIdx) {
        if (outPath) {
            outPath->clear();
            outPath->push_back(start);
        }
        return 0;
    }

    // ---------------------------------------------------------
    // 2. 状态管理
    // ---------------------------------------------------------

    size_t requiredSize = (size_t)map.width * (size_t)map.height;
    if (nodeData.size() != requiredSize) {
        nodeData.resize(requiredSize);
        openSetContainer.reserve(requiredSize / 8);
        currentGen = 0;
    }

    if (currentGen == 0) {
        std::fill(nodeData.begin(), nodeData.end(), NodeData());
        currentGen = 1;
    } else {
        currentGen++;
        if (currentGen == 0) {
            std::fill(nodeData.begin(), nodeData.end(), NodeData());
            currentGen = 1;
        }
    }

    openSetContainer.clear();

    // ---------------------------------------------------------
    // 3. 核心计算 Lambda
    // ---------------------------------------------------------

    auto heuristic = [&](const Hex& a, const Hex& b) -> int64_t {
        return (int64_t)a.distance(b) * STEP_COST;
    };

    // 预计算起终点向量 (Offset Space)
    const int64_t dCol = (int64_t)eCol - sCol;
    const int64_t dRow = (int64_t)eRow - sRow;

    // 【优化】直接传入 (col, row)，利用外层算好的值，避免 hexToOffset 重复计算
    auto getTieBreaker = [&](int c, int r) -> int64_t {
        int64_t currDCol = (int64_t)c - sCol;
        int64_t currDRow = (int64_t)r - sRow;
        return std::llabs(dCol * currDRow - currDCol * dRow);
    };

    // ---------------------------------------------------------
    // 4. A* 主循环
    // ---------------------------------------------------------

    nodeData[startIdx].visitedGen = currentGen;
    nodeData[startIdx].gScore = 0;
    nodeData[startIdx].parentIdx = -1;

    openSetContainer.push_back({
        heuristic(start, end),
        0,
        getTieBreaker(sCol, sRow), // 传入 Offset 坐标
        startIdx
    });
    std::push_heap(openSetContainer.begin(), openSetContainer.end(), NodeComparator());

    int steps = 0;

    while (!openSetContainer.empty()) {
        if (maxSteps != -1 && steps++ >= maxSteps) return -1;

        std::pop_heap(openSetContainer.begin(), openSetContainer.end(), NodeComparator());
        SearchNode current = openSetContainer.back();
        openSetContainer.pop_back();

        // Lazy Deletion 检查
        if (nodeData[current.index].visitedGen != currentGen) continue;
        if (current.gScore != nodeData[current.index].gScore) continue;

        int idx = current.index;

        // --- 找到终点 ---
        if (idx == endIdx) {
            if (outPath) {
                outPath->clear();
                if (nodeData[endIdx].gScore > 0) {
                    outPath->reserve((size_t)(nodeData[endIdx].gScore / STEP_COST) + 4);
                }

                int curr = endIdx;
                while (curr != -1) {
                    // 反算 Hex
                    int cr = curr / map.width;
                    int cc = curr % map.width;
                    int cq = cc - (cr - (cr & 1)) / 2;
                    outPath->push_back(Hex(cq, cr));

                    if (curr == startIdx) break;
                    curr = nodeData[curr].parentIdx;
                }
                std::reverse(outPath->begin(), outPath->end());
            }
            return nodeData[endIdx].gScore;
        }

        // --- 扩展邻居 ---

        // 反算当前 Hex (只用于计算邻居方向，TieBreaker 用 Offset)
        int currR = idx / map.width;
        int currC = idx % map.width;
        int currQ = currC - (currR - (currR & 1)) / 2;
        Hex currentHex(currQ, currR);

        VIZ_LOG(currentHex);

        for (const auto& dir : HEX_DIRS) {
            Hex next = currentHex + dir;

            // 手动计算 Offset 坐标 (Odd-R)，用于 Index 和 TieBreaker
            int nc = next.q + (next.r - (next.r & 1)) / 2;
            int nr = next.r;

            // 快速边界检查
            if (nc < 0 || nc >= map.width || nr < 0 || nr >= map.height) continue;

            int nextIdx = nr * map.width + nc;

            if (!map.isWalkable(next)) continue;
            if (clusterId != -1 && map.getClusterId(next) != clusterId) continue;

            int64_t newG = nodeData[idx].gScore + STEP_COST;

            if (nodeData[nextIdx].visitedGen != currentGen || newG < nodeData[nextIdx].gScore) {
                nodeData[nextIdx].visitedGen = currentGen;
                nodeData[nextIdx].gScore = newG;
                nodeData[nextIdx].parentIdx = idx;

                openSetContainer.push_back({
                    newG + heuristic(next, end),
                    newG,
                    getTieBreaker(nc, nr), // 【核心优化】直接传入 nc, nr
                    nextIdx
                });
                std::push_heap(openSetContainer.begin(), openSetContainer.end(), NodeComparator());
            }
        }
    }

    return -1;
}