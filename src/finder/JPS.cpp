//JSP.cpp
#include "JPS.h"
#include <cmath>
#include <algorithm>
#include "../common/VizMacros.h"

// ============================================================================
// 初始化与重置
// ============================================================================
JPS::JPS(HexMap& m) : map(m) {
    ensureCapacity();
}

void JPS::ensureCapacity() {
    int w = map.width;
    int h = map.height;
    int size = w * h;

    if (w == lastWidth && h == lastHeight && size == mapSize) return;

    lastWidth = w;
    lastHeight = h;
    mapSize = size;

    // 重新分配内存
    neighborIndex.assign(static_cast<size_t>(size) * 6, -1);
    coordQ.resize(size);
    coordR.resize(size);
    nodeData.assign(size, Node{});

    // 缓存扩容
    jumpCacheGen.assign(static_cast<size_t>(size) * 6, 0);
    jumpCacheResult.assign(static_cast<size_t>(size) * 6, -1);

    // 预留 OpenList 空间
    openList.reserve(1024);

    rebuildGeometryTables();
}

void JPS::rebuildGeometryTables() {
    int w = map.width;

    // 1. 构建坐标映射 (Offset -> Axial)
    // 适配 Odd-r (0,0 Bottom-Left)
    for (int idx = 0; idx < mapSize; ++idx) {
        int r = idx / w; // row
        int c = idx % w; // col

        coordR[idx] = r;
        coordQ[idx] = c - (r - (r & 1)) / 2;
    }

    // 2. 构建邻居表 (基于 Axial 向量 DIR_DQ/DR)
    // 这保证了 heuristic 和 neighbor 移动是几何上完全一致的
    for (int idx = 0; idx < mapSize; ++idx) {
        int q = coordQ[idx];
        int r = coordR[idx];

        for (int dir = 0; dir < 6; ++dir) {
            int nq = q + DIR_DQ[dir];
            int nr = r + DIR_DR[dir];

            // Axial -> Odd-r Offset
            int nc = nq + (nr - (nr & 1)) / 2;
            int nr_offset = nr; // row is r

            if (nc >= 0 && nc < lastWidth && nr_offset >= 0 && nr_offset < lastHeight) {
                neighborIndex[idx * 6 + dir] = nr_offset * lastWidth + nc;
            } else {
                neighborIndex[idx * 6 + dir] = -1;
            }
        }
    }
}

void JPS::clearAllNodeGens() {
    // 彻底重置所有节点状态，通常只在 generation 溢出时调用
    for (auto& n : nodeData) {
        n.gScore = std::numeric_limits<int64_t>::max() / 2;
        n.parentIdx = -1;
        n.openGen = 0;
        n.closeGen = 0;
        n.arrivalDir = -1;
    }
}

void JPS::reset() {
    // 简单的重置只需增加代数
    ++currentGen;
    if (currentGen == 0) {
        // 溢出处理
        std::fill(jumpCacheGen.begin(), jumpCacheGen.end(), 0u);
        clearAllNodeGens();
        currentGen = 1;
    }
    openList.clear();
}

// ============================================================================
// JPS 核心逻辑
// ============================================================================

bool JPS::hasForcedNeighborCardinal(int idx, int dirIdx) const {
    // 检查方向 d 两侧的 "后方阻挡 + 前方可通"
    // d+2 (左后), d+1 (左前)
    // d-2 (右后), d-1 (右前)

    int base = idx * 6;

    // 优化：直接展开计算方向，减少 wrapDir 调用
    // 假定 dirIdx 是 0,2,4 (Cardinal)
    // 左侧: d+2, d+1
    int dLeftBack = (dirIdx + 2) % 6;
    int dLeftFore = (dirIdx + 1) % 6; // Cardinal+1 必为 Intercardinal，不需要wrap负数

    // 右侧: d-2, d-1.  (0-2)=-2 -> 4, (0-1)=-1 -> 5
    int dRightBack = (dirIdx >= 2) ? (dirIdx - 2) : (dirIdx + 4);
    int dRightFore = (dirIdx >= 1) ? (dirIdx - 1) : (dirIdx + 5);

    int lbIdx = neighborIndex[base + dLeftBack];
    bool lbBlocked = (lbIdx < 0) || !map.isWalkableIdx(lbIdx);

    if (lbBlocked) {
        int lfIdx = neighborIndex[base + dLeftFore];
        if (lfIdx >= 0 && map.isWalkableIdx(lfIdx)) return true;
    }

    int rbIdx = neighborIndex[base + dRightBack];
    bool rbBlocked = (rbIdx < 0) || !map.isWalkableIdx(rbIdx);

    if (rbBlocked) {
        int rfIdx = neighborIndex[base + dRightFore];
        if (rfIdx >= 0 && map.isWalkableIdx(rfIdx)) return true;
    }

    return false;
}

uint8_t JPS::getSuccessorDirsMask(int idx, int parentDir) const {
    if (parentDir == -1) return 0x3F; // 起点：所有方向

    uint8_t mask = 0;

    if (isCardinal(parentDir)) {
        // 1. 保持当前直线方向
        mask |= (1u << parentDir);

        // 2. 只有在有强制邻居时，才添加对应的转弯方向
        // 强制邻居逻辑：
        // 左后(d+2)堵 & 左前(d+1)通 -> 强制去 d+1
        // 右后(d-2)堵 & 右前(d-1)通 -> 强制去 d-1

        int base = idx * 6;

        int dLeftBack = (parentDir + 2) % 6;
        int lbIdx = neighborIndex[base + dLeftBack];
        if (lbIdx < 0 || !map.isWalkableIdx(lbIdx)) {
             int dLeftFore = (parentDir + 1) % 6;
             int lfIdx = neighborIndex[base + dLeftFore];
             if (lfIdx >= 0 && map.isWalkableIdx(lfIdx)) {
                 mask |= (1u << dLeftFore);
             }
        }

        int dRightBack = (parentDir >= 2) ? (parentDir - 2) : (parentDir + 4);
        int rbIdx = neighborIndex[base + dRightBack];
        if (rbIdx < 0 || !map.isWalkableIdx(rbIdx)) {
            int dRightFore = (parentDir >= 1) ? (parentDir - 1) : (parentDir + 5);
            int rfIdx = neighborIndex[base + dRightFore];
            if (rfIdx >= 0 && map.isWalkableIdx(rfIdx)) {
                mask |= (1u << dRightFore);
            }
        }

    } else {
        // Intercardinal (组合方向)
        // 自然邻居：前进方向 + 两个分量方向 (d-1, d+1)
        mask |= (1u << parentDir);
        mask |= (1u << ((parentDir + 5) % 6)); // d-1
        mask |= (1u << ((parentDir + 1) % 6)); // d+1
    }

    return mask;
}

// ============================================================================
// 跳跃逻辑
// ============================================================================

int JPS::jumpCardinal(int fromIdx, int dirIdx, int endIdx) {
    size_t key = static_cast<size_t>(fromIdx) * 6 + dirIdx;

    if (jumpCacheGen[key] == currentGen) {
        return jumpCacheResult[key];
    }

    int cur = getWalkableNeighbor(fromIdx, dirIdx);
    // 缓存 -1 结果
    if (cur == -1) {
        jumpCacheGen[key] = currentGen;
        jumpCacheResult[key] = -1;
        return -1;
    }

    while (true) {
        if (cur == endIdx) {
            jumpCacheGen[key] = currentGen;
            jumpCacheResult[key] = cur;
            return cur;
        }

        // 检查强制邻居
        if (hasForcedNeighborCardinal(cur, dirIdx)) {
            jumpCacheGen[key] = currentGen;
            jumpCacheResult[key] = cur;
            return cur;
        }

        int next = getWalkableNeighbor(cur, dirIdx);
        if (next == -1) {
            jumpCacheGen[key] = currentGen;
            jumpCacheResult[key] = -1;
            return -1;
        }
        cur = next;
    }
}

int JPS::jumpIntercardinal(int fromIdx, int dirIdx, int endIdx) {
    size_t key = static_cast<size_t>(fromIdx) * 6 + dirIdx;

    if (jumpCacheGen[key] == currentGen) {
        return jumpCacheResult[key];
    }

    int cur = getWalkableNeighbor(fromIdx, dirIdx);
    if (cur == -1) {
        jumpCacheGen[key] = currentGen;
        jumpCacheResult[key] = -1;
        return -1;
    }

    // 分解为两个 Cardinal 分量
    int d1 = (dirIdx + 5) % 6; // dir - 1
    int d2 = (dirIdx + 1) % 6; // dir + 1

    while (true) {
        if (cur == endIdx) {
            jumpCacheGen[key] = currentGen;
            jumpCacheResult[key] = cur;
            return cur;
        }

        // 如果任一分量方向能产生跳点，则当前点也是跳点
        // 注意：这里递归调用 jumpCardinal，它会利用缓存，不会太慢
        if (jumpCardinal(cur, d1, endIdx) != -1) {
            jumpCacheGen[key] = currentGen;
            jumpCacheResult[key] = cur;
            return cur;
        }
        if (jumpCardinal(cur, d2, endIdx) != -1) {
            jumpCacheGen[key] = currentGen;
            jumpCacheResult[key] = cur;
            return cur;
        }

        int next = getWalkableNeighbor(cur, dirIdx);
        if (next == -1) {
            jumpCacheGen[key] = currentGen;
            jumpCacheResult[key] = -1;
            return -1;
        }
        cur = next;
    }
}

int JPS::jumpIdx(int fromIdx, int dirIdx, int endIdx) {
    if (isCardinal(dirIdx)) {
        return jumpCardinal(fromIdx, dirIdx, endIdx);
    } else {
        return jumpIntercardinal(fromIdx, dirIdx, endIdx);
    }
}

// ============================================================================
// 主寻路函数
// ============================================================================

std::vector<Hex> JPS::findPath(Hex start, Hex end) {
    ensureCapacity();

    int startIdx = map.getIndex(start);
    int endIdx = map.getIndex(end);

    if (startIdx < 0 || endIdx < 0 || startIdx >= mapSize || endIdx >= mapSize) return {};
    if (!map.isWalkableIdx(startIdx) || !map.isWalkableIdx(endIdx)) return {};
    if (startIdx == endIdx) return { start };

    // Reset Gen
    ++currentGen;
    if (currentGen == 0) {
        std::fill(jumpCacheGen.begin(), jumpCacheGen.end(), 0u);
        clearAllNodeGens();
        currentGen = 1;
    }
    openList.clear();

    // Init Start Node
    Node& startNode = nodeData[startIdx];
    startNode.gScore = 0;
    startNode.parentIdx = -1;
    startNode.openGen = currentGen;
    startNode.closeGen = 0;
    startNode.arrivalDir = -1; // 起点没有来源方向

    // Push Start
    openList.push_back({ hexDistIdx(startIdx, endIdx), 0, startIdx });
    std::push_heap(openList.begin(), openList.end(), std::greater<PQItem>());

    while (!openList.empty()) {
        // Pop best
        std::pop_heap(openList.begin(), openList.end(), std::greater<PQItem>());
        PQItem item = openList.back();
        openList.pop_back();

        int currIdx = item.idx;

        // Lazy cleanup check
        if (nodeData[currIdx].openGen != currentGen) continue;
        if (nodeData[currIdx].closeGen == currentGen) continue;
        if (item.g > nodeData[currIdx].gScore) continue;

        nodeData[currIdx].closeGen = currentGen;

        // Viz / Debug
        VIZ_LOG(map.getHex(currIdx));

        if (currIdx == endIdx) {
            return reconstructPath(endIdx);
        }

        // 优化点：直接使用缓存的 arrivalDir，无需计算 getDirectionIndex
        int parentDir = nodeData[currIdx].arrivalDir;

        // 计算后继方向
        uint8_t mask = getSuccessorDirsMask(currIdx, parentDir);

        for (int dir = 0; dir < 6; ++dir) {
            if (!(mask & (1u << dir))) continue;

            // 第一步检查：如果立刻受阻，无需跳跃
            if (getWalkableNeighbor(currIdx, dir) == -1) continue;

            int jpIdx = jumpIdx(currIdx, dir, endIdx);

            if (jpIdx != -1) {
                // 发现跳点
                Node& jpNode = nodeData[jpIdx];

                // 懒初始化
                if (jpNode.openGen != currentGen) {
                    jpNode.openGen = currentGen;
                    jpNode.closeGen = 0;
                    jpNode.gScore = std::numeric_limits<int64_t>::max() / 2;
                    jpNode.parentIdx = -1;
                }

                if (jpNode.closeGen == currentGen) continue;

                int64_t newG = nodeData[currIdx].gScore + hexDistIdx(currIdx, jpIdx);

                if (newG < jpNode.gScore) {
                    jpNode.gScore = newG;
                    jpNode.parentIdx = currIdx;
                    jpNode.arrivalDir = dir; // 记录到达方向！关键优化！

                    int64_t h = hexDistIdx(jpIdx, endIdx);
                    openList.push_back({ newG + h, newG, jpIdx });
                    std::push_heap(openList.begin(), openList.end(), std::greater<PQItem>());
                }
            }
        }
    }

    return {};
}

// ============================================================================
// 辅助函数
// ============================================================================

int JPS::getDirectionIndex(int fromIdx, int toIdx) const {
    // 仅在 reconstructPath 中使用，无需极致优化
    if (fromIdx == toIdx) return -1;

    int dq = coordQ[toIdx] - coordQ[fromIdx];
    int dr = coordR[toIdx] - coordR[fromIdx];
    int ds = -dq - dr;

    // 计算距离
    int dist = (std::abs(dq) + std::abs(dr) + std::abs(ds)) >> 1;
    if (dist == 0) return -1;

    int ndq = dq / dist;
    int ndr = dr / dist;

    for(int i=0; i<6; ++i) {
        if(DIR_DQ[i] == ndq && DIR_DR[i] == ndr) return i;
    }
    return -1;
}

std::vector<Hex> JPS::reconstructPath(int endIdx) const {
    std::vector<int> pathIndices;
    int curr = endIdx;

    while (curr != -1) {
        pathIndices.push_back(curr);
        curr = nodeData[curr].parentIdx;
    }

    if (pathIndices.empty()) return {};
    std::reverse(pathIndices.begin(), pathIndices.end());

    std::vector<Hex> fullPath;
    // 预估大小
    fullPath.reserve(pathIndices.size() * 5);

    fullPath.push_back(map.getHex(pathIndices[0]));

    for (size_t i = 0; i < pathIndices.size() - 1; ++i) {
        int from = pathIndices[i];
        int to = pathIndices[i+1];

        int dir = getDirectionIndex(from, to);

        // 填充中间点
        if (dir != -1) {
            int step = neighborIndex[from * 6 + dir];
            while (step != -1 && step != to) {
                fullPath.push_back(map.getHex(step));
                step = neighborIndex[step * 6 + dir];
            }
        }
        fullPath.push_back(map.getHex(to));
    }

    return fullPath;
}