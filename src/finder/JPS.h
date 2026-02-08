// JSP.h
#pragma once

#include "IFinder.h"
#include "../map/HexMap.h"
#include <vector>
#include <cstdint>
#include <limits>
#include <algorithm>

class JPS : public IFinder {
public:
    explicit JPS(HexMap& m);
    ~JPS() override = default;

    std::string getName() const override { return "JPS"; }
    std::vector<Hex> findPath(Hex start, Hex end) override;
    void reset() override;

    // 当地图障碍物改变时调用
    void notifyObstacleChanged() { ++currentGen; }

private:
    HexMap& map;

    // ==================== 几何常量与查找表 ====================
    // 6个方向的 Axial 向量 delta (q, r)
    // 顺序：E, NE, NW, W, SW, SE (基于 Odd-r 左下角逻辑适配)
    static constexpr int DIR_DQ[6] = { 1,  1,  0, -1, -1,  0 };
    static constexpr int DIR_DR[6] = { 0, -1, -1,  0,  1,  1 };

    // [idx * 6 + dir] -> 邻居的 idx，-1 表示越界
    std::vector<int> neighborIndex;

    // 预计算坐标，用于快速计算 H 值
    std::vector<int> coordQ;
    std::vector<int> coordR;

    int lastWidth = 0;
    int lastHeight = 0;
    int mapSize = 0;

    // ==================== 寻路数据结构 ====================
    struct Node {
        int64_t  gScore    = std::numeric_limits<int64_t>::max() / 2;
        int      parentIdx = -1;
        uint32_t openGen   = 0;
        uint32_t closeGen  = 0;
        int8_t   arrivalDir = -1; // 优化：记录到达该节点的方向 (0-5)，避免反向计算
    };
    std::vector<Node> nodeData;

    // OpenList 内存复用
    struct PQItem {
        int64_t f; // 优先级 (f-score)
        int64_t g; // tie-breaker (g-score)
        int idx;

        // 最小堆：f 越小越优先；f 相同 g 越大越优先（倾向更深搜索）
        bool operator>(const PQItem& other) const {
            return f != other.f ? f > other.f : g < other.g;
        }
    };
    std::vector<PQItem> openList;

    // ==================== JPS 缓存 ====================
    // jumpCacheResult: 存储跳跃结果 idx
    // jumpCacheGen: 存储该结果属于哪一代
    std::vector<uint32_t> jumpCacheGen;
    std::vector<int>      jumpCacheResult;
    uint32_t currentGen = 1;

    // ==================== 内部函数 ====================
    void ensureCapacity();
    void rebuildGeometryTables();
    void clearAllNodeGens();

    static inline int wrapDir(int d) {
        return (d >= 6) ? (d - 6) : (d < 0 ? d + 6 : d);
    }

    // 偶数方向(0,2,4)为直线(Cardinal)，奇数方向(1,3,5)为组合(Intercardinal)
    static inline bool isCardinal(int dir) { return (dir & 1) == 0; }

    inline int getNeighbor(int idx, int dir) const {
        return neighborIndex[idx * 6 + dir];
    }

    // 获取可通行邻居，越界或障碍均返回 -1
    inline int getWalkableNeighbor(int idx, int dir) const {
        int n = neighborIndex[idx * 6 + dir];
        // 这里假设 map.isWalkableIdx 很快（通常是数组访问）
        return (n >= 0 && map.isWalkableIdx(n)) ? n : -1;
    }

    // 启发式距离 (Axial Manhattan Distance)
    inline int64_t hexDistIdx(int aIdx, int bIdx) const {
        int dq = coordQ[bIdx] - coordQ[aIdx];
        int dr = coordR[bIdx] - coordR[aIdx];
        // 隐式的 ds = -dq - dr
        // 距离公式：(abs(dq) + abs(dr) + abs(dq+dr)) / 2
        // 下面是展开后的优化写法
        return (std::abs(dq) + std::abs(dr) + std::abs(dq + dr)) >> 1;
    }

    // 核心 JPS 逻辑
    int jumpIdx(int fromIdx, int dirIdx, int endIdx);
    int jumpCardinal(int fromIdx, int dirIdx, int endIdx);
    int jumpIntercardinal(int fromIdx, int dirIdx, int endIdx);

    bool hasForcedNeighborCardinal(int idx, int dirIdx) const;
    uint8_t getSuccessorDirsMask(int idx, int parentDir) const;

    // 辅助路径重建
    int getDirectionIndex(int fromIdx, int toIdx) const;
    std::vector<Hex> reconstructPath(int endIdx) const;
};