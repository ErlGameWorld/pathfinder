//HJPS.cpp
#include "HJPS.h"
#include <cmath>
#include <algorithm>
#include "../common/VizMacros.h"

HJPS::HJPS(HexMap& m) : map(m) {
    ensureCapacity();
}

void HJPS::ensureCapacity() {
    int w = map.width;
    int h = map.height;
    int size = w * h;

    if (w == lastWidth && h == lastHeight && size == mapSize) return;

    lastWidth = w;
    lastHeight = h;
    mapSize = size;

    neighborIndex.assign(static_cast<size_t>(size) * 6, -1);
    coordQ.resize(size);
    coordR.resize(size);
    nodeData.assign(size, Node{});

    jumpCacheGen.assign(static_cast<size_t>(size) * 6, 0);
    jumpCacheResult.assign(static_cast<size_t>(size) * 6, -1);

    openList.reserve(1024);

    rebuildGeometryTables();
}

void HJPS::rebuildGeometryTables() {
    // ---------------------------------------------------------
    // 关键修正：针对 Odd-r (Y-Up, Bottom-Left) 建立精确映射
    // ---------------------------------------------------------

    // 1. 建立 Axial 坐标 (用于 Heuristic)
    // 转换公式: q = x - (y - (y&1)) / 2, r = y
    for (int idx = 0; idx < mapSize; ++idx) {
        int r = idx / lastWidth; // y
        int c = idx % lastWidth; // x

        coordR[idx] = r;
        coordQ[idx] = c - (r - (r & 1)) / 2;
    }

    // 2. 建立 Offset 物理邻居表
    // 必须确保这里的 0-5 方向与 DIR_DQ/DR 完全一致
    // 0: E, 1: SE, 2: SW, 3: W, 4: NW, 5: NE

    for (int y = 0; y < lastHeight; ++y) {
        for (int x = 0; x < lastWidth; ++x) {
            int idx = y * lastWidth + x;
            int parity = y & 1; // 1 for Odd Row, 0 for Even Row

            int neighbors[6][2];

            // Dir 0 (E): (+1, 0)
            neighbors[0][0] = x + 1; neighbors[0][1] = y;

            // Dir 1 (SE): Y-1. Odd(x+1), Even(x)
            neighbors[1][0] = x + (parity ? 1 : 0); neighbors[1][1] = y - 1;

            // Dir 2 (SW): Y-1. Odd(x), Even(x-1)
            neighbors[2][0] = x + (parity ? 0 : -1); neighbors[2][1] = y - 1;

            // Dir 3 (W): (-1, 0)
            neighbors[3][0] = x - 1; neighbors[3][1] = y;

            // Dir 4 (NW): Y+1. Odd(x), Even(x-1)
            neighbors[4][0] = x + (parity ? 0 : -1); neighbors[4][1] = y + 1;

            // Dir 5 (NE): Y+1. Odd(x+1), Even(x)
            neighbors[5][0] = x + (parity ? 1 : 0); neighbors[5][1] = y + 1;

            for (int dir = 0; dir < 6; ++dir) {
                int nx = neighbors[dir][0];
                int ny = neighbors[dir][1];
                if (nx >= 0 && nx < lastWidth && ny >= 0 && ny < lastHeight) {
                    neighborIndex[idx * 6 + dir] = ny * lastWidth + nx;
                } else {
                    neighborIndex[idx * 6 + dir] = -1;
                }
            }
        }
    }
}

void HJPS::clearAllNodeGens() {
    for (auto& n : nodeData) {
        n.gScore = std::numeric_limits<int64_t>::max() / 2;
        n.parentIdx = -1;
        n.openGen = 0;
        n.closeGen = 0;
        n.arrivalDir = -1;
    }
}

void HJPS::reset() {
    ++currentGen;
    if (currentGen == 0) {
        std::fill(jumpCacheGen.begin(), jumpCacheGen.end(), 0u);
        clearAllNodeGens();
        currentGen = 1;
    }
    openList.clear();
}

// ============================================================================
// HJPS 逻辑 (原版正确逻辑)
// ============================================================================

bool HJPS::hasForcedNeighborCardinal(int idx, int dirIdx) const {
    // 检查 Cardinal 方向两侧的 "后方阻挡 + 前方可通"
    // 对于 Cardinal 方向 d：
    // 左侧检测: d+2 (Backward), d+1 (Forward-Diagonal)
    // 右侧检测: d-2 (Backward), d-1 (Forward-Diagonal)

    int base = idx * 6;

    // Left Side Check
    int dLeftBack = (dirIdx + 2) % 6;
    int lbIdx = neighborIndex[base + dLeftBack];
    bool lbBlocked = (lbIdx < 0) || !map.isWalkableIdx(lbIdx);

    if (lbBlocked) {
        int dLeftFore = (dirIdx + 1) % 6; // Cardinal+1 -> Intercardinal
        int lfIdx = neighborIndex[base + dLeftFore];
        if (lfIdx >= 0 && map.isWalkableIdx(lfIdx)) return true;
    }

    // Right Side Check
    int dRightBack = (dirIdx + 4) % 6; // -2
    int rbIdx = neighborIndex[base + dRightBack];
    bool rbBlocked = (rbIdx < 0) || !map.isWalkableIdx(rbIdx);

    if (rbBlocked) {
        int dRightFore = (dirIdx + 5) % 6; // -1
        int rfIdx = neighborIndex[base + dRightFore];
        if (rfIdx >= 0 && map.isWalkableIdx(rfIdx)) return true;
    }

    return false;
}

uint8_t HJPS::getSuccessorDirsMask(int idx, int parentDir) const {
    if (parentDir == -1) return 0x3F; // 起点：所有方向

    uint8_t mask = 0;

    if (isCardinal(parentDir)) {
        // 1. 保持当前直线方向
        mask |= (1u << parentDir);

        // 2. 强制邻居逻辑
        int base = idx * 6;

        // Check Left (dir+2 blocked -> go dir+1)
        int dLeftBack = (parentDir + 2) % 6;
        int lbIdx = neighborIndex[base + dLeftBack];
        if (lbIdx < 0 || !map.isWalkableIdx(lbIdx)) {
             int dLeftFore = (parentDir + 1) % 6;
             int lfIdx = neighborIndex[base + dLeftFore];
             if (lfIdx >= 0 && map.isWalkableIdx(lfIdx)) {
                 mask |= (1u << dLeftFore);
             }
        }

        // Check Right (dir-2 blocked -> go dir-1)
        int dRightBack = (parentDir + 4) % 6;
        int rbIdx = neighborIndex[base + dRightBack];
        if (rbIdx < 0 || !map.isWalkableIdx(rbIdx)) {
            int dRightFore = (parentDir + 5) % 6;
            int rfIdx = neighborIndex[base + dRightFore];
            if (rfIdx >= 0 && map.isWalkableIdx(rfIdx)) {
                mask |= (1u << dRightFore);
            }
        }

    } else {
        // Intercardinal (组合方向)
        // 自然邻居：前进方向(d) + 两个分量方向 (d-1, d+1)
        // 因为 d 是奇数，d-1 和 d+1 必然是 Cardinal
        mask |= (1u << parentDir);
        mask |= (1u << ((parentDir + 5) % 6)); // d-1
        mask |= (1u << ((parentDir + 1) % 6)); // d+1
    }

    return mask;
}

int HJPS::jumpCardinal(int fromIdx, int dirIdx, int endIdx) {
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

    while (true) {
        if (cur == endIdx) {
            jumpCacheGen[key] = currentGen;
            jumpCacheResult[key] = cur;
            return cur;
        }

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

int HJPS::jumpIntercardinal(int fromIdx, int dirIdx, int endIdx) {
    // Intercardinal logic: Move 1 step, then check Cardinal components
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

        // 关键逻辑：在每个步进点，向两个分量方向发射 Cardinal 射线
        // 这覆盖了空地
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

int HJPS::jumpIdx(int fromIdx, int dirIdx, int endIdx) {
    if (isCardinal(dirIdx)) {
        return jumpCardinal(fromIdx, dirIdx, endIdx);
    } else {
        return jumpIntercardinal(fromIdx, dirIdx, endIdx);
    }
}

std::vector<Hex> HJPS::findPath(Hex start, Hex end) {
    ensureCapacity();

    int startIdx = map.getIndex(start);
    int endIdx = map.getIndex(end);

    if (startIdx < 0 || endIdx < 0 || startIdx >= mapSize || endIdx >= mapSize) return {};
    if (!map.isWalkableIdx(startIdx) || !map.isWalkableIdx(endIdx)) return {};
    if (startIdx == endIdx) return { start };

    reset();

    Node& startNode = nodeData[startIdx];
    startNode.gScore = 0;
    startNode.openGen = currentGen;
    startNode.arrivalDir = -1;

    openList.push_back({ hexDistIdx(startIdx, endIdx), 0, startIdx });
    std::push_heap(openList.begin(), openList.end(), std::greater<PQItem>());

    while (!openList.empty()) {
        std::pop_heap(openList.begin(), openList.end(), std::greater<PQItem>());
        PQItem item = openList.back();
        openList.pop_back();

        int currIdx = item.idx;

        if (nodeData[currIdx].openGen != currentGen) continue;
        if (nodeData[currIdx].closeGen == currentGen) continue;
        if (item.g > nodeData[currIdx].gScore) continue;

        nodeData[currIdx].closeGen = currentGen;
        VIZ_LOG(map.getHex(currIdx));

        if (currIdx == endIdx) return reconstructPath(endIdx);

        int parentDir = nodeData[currIdx].arrivalDir;
        uint8_t mask = getSuccessorDirsMask(currIdx, parentDir);

        for (int dir = 0; dir < 6; ++dir) {
            if (!(mask & (1u << dir))) continue;

            // 无需在此处检查 getWalkableNeighbor，jumpIdx 内部第一步会检查
            int jpIdx = jumpIdx(currIdx, dir, endIdx);

            if (jpIdx != -1) {
                Node& jpNode = nodeData[jpIdx];

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
                    jpNode.arrivalDir = dir;

                    int64_t h = hexDistIdx(jpIdx, endIdx);
                    openList.push_back({ newG + h, newG, jpIdx });
                    std::push_heap(openList.begin(), openList.end(), std::greater<PQItem>());
                }
            }
        }
    }

    return {};
}

std::vector<Hex> HJPS::reconstructPath(int endIdx) const {
    std::vector<int> pathIndices;
    int curr = endIdx;

    while (curr != -1) {
        pathIndices.push_back(curr);
        curr = nodeData[curr].parentIdx;
    }

    if (pathIndices.empty()) return {};
    std::reverse(pathIndices.begin(), pathIndices.end());

    std::vector<Hex> fullPath;
    fullPath.reserve(pathIndices.size() * 5);
    fullPath.push_back(map.getHex(pathIndices[0]));

    for (size_t i = 0; i < pathIndices.size() - 1; ++i) {
        int from = pathIndices[i];
        int to = pathIndices[i+1];

        // 使用 nodeData 缓存的方向来重建，避免几何计算误差
        int dir = nodeData[to].arrivalDir;

        // 如果是起点的第一步，或者数据异常，尝试计算方向
        if (dir == -1) {
            // 简单遍历 6 个邻居找 to
            for(int d=0; d<6; ++d) {
                // 这里我们要寻找的是长距离跳跃的方向，
                // 所以我们要找那个方向，使得 neighborIndex[from][d] 是走向 to 的第一步
                // 由于跳点之间是直线的，可以用 Axial 坐标差来算
                int dq = coordQ[to] - coordQ[from];
                int dr = coordR[to] - coordR[from];
                // 计算单位向量
                int dist = (std::abs(dq) + std::abs(dr) + std::abs(dq+dr)) >> 1;
                if (dist > 0) {
                   if (DIR_DQ[d] == dq/dist && DIR_DR[d] == dr/dist) {
                       dir = d;
                       break;
                   }
                }
            }
        }

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