//HJPS.h
#pragma once

#include "IFinder.h"
#include "../map/HexMap.h"
#include <vector>
#include <cstdint>
#include <limits>
#include <algorithm>

class HJPS : public IFinder {
public:
    explicit HJPS(HexMap& m);
    ~HJPS() override = default;

    std::string getName() const override { return "HJPS"; }
    std::vector<Hex> findPath(Hex start, Hex end) override;
    void reset() override;

    void notifyObstacleChanged() { ++currentGen; }

private:
    HexMap& map;

    // ==================== 几何常量 ====================
    // Axial 向量定义 (配合 Odd-r Y-Up)
    // 0: E  (+1, 0) -> Cardinal
    // 1: SE (+1, -1)-> Intercardinal (Sum of 0 & 2)
    // 2: SW ( 0, -1)-> Cardinal
    // 3: W  (-1, 0) -> Intercardinal (Sum of 2 & 4)
    // 4: NW (-1, +1)-> Cardinal
    // 5: NE ( 0, +1)-> Intercardinal (Sum of 4 & 0)
    static constexpr int DIR_DQ[6] = { 1,  1,  0, -1, -1,  0 };
    static constexpr int DIR_DR[6] = { 0, -1, -1,  0,  1,  1 };

    std::vector<int> neighborIndex; // Lookup Table
    std::vector<int> coordQ;
    std::vector<int> coordR;

    int lastWidth = 0;
    int lastHeight = 0;
    int mapSize = 0;

    // ==================== 节点数据 ====================
    struct Node {
        int64_t  gScore    = std::numeric_limits<int64_t>::max() / 2;
        int      parentIdx = -1;
        uint32_t openGen   = 0;
        uint32_t closeGen  = 0;
        int8_t   arrivalDir = -1;
    };
    std::vector<Node> nodeData;

    struct PQItem {
        int64_t f;
        int64_t g;
        int idx;
        bool operator>(const PQItem& other) const {
            return f != other.f ? f > other.f : g < other.g;
        }
    };
    std::vector<PQItem> openList;

    // ==================== 缓存 ====================
    std::vector<uint32_t> jumpCacheGen;
    std::vector<int>      jumpCacheResult;
    uint32_t currentGen = 1;

    // ==================== 核心函数 ====================
    void ensureCapacity();
    void rebuildGeometryTables();
    void clearAllNodeGens();

    // 偶数方向(0,2,4)为轴向(Cardinal)，奇数方向(1,3,5)为组合向(Intercardinal)
    static inline bool isCardinal(int dir) { return (dir & 1) == 0; }

    inline int getNeighbor(int idx, int dir) const {
        return neighborIndex[idx * 6 + dir];
    }

    inline int getWalkableNeighbor(int idx, int dir) const {
        int n = neighborIndex[idx * 6 + dir];
        return (n >= 0 && map.isWalkableIdx(n)) ? n : -1;
    }

    // Axial Distance
    inline int64_t hexDistIdx(int aIdx, int bIdx) const {
        int dq = coordQ[bIdx] - coordQ[aIdx];
        int dr = coordR[bIdx] - coordR[aIdx];
        return (std::abs(dq) + std::abs(dr) + std::abs(dq + dr)) >> 1;
    }

    int jumpIdx(int fromIdx, int dirIdx, int endIdx);
    int jumpCardinal(int fromIdx, int dirIdx, int endIdx);
    int jumpIntercardinal(int fromIdx, int dirIdx, int endIdx);

    bool hasForcedNeighborCardinal(int idx, int dirIdx) const;
    uint8_t getSuccessorDirsMask(int idx, int parentDir) const;

    std::vector<Hex> reconstructPath(int endIdx) const;
};