#pragma once
#include "IFinder.h"
#include "../map/HexMap.h"
#include <vector>
#include <queue>
#include <cstdint>
#include <string>

class HJPS : public IFinder {
    HexMap& map;

    struct Node {
        uint32_t visitedGen = 0;
        uint32_t closedGen  = 0;
        int64_t  gScore     = INT64_MAX;
        int32_t  parentIdx  = -1;
        int8_t   parentDir  = -1;
    };

    std::vector<Node> nodes;
    uint32_t currentGen = 0;

public:
    explicit HJPS(HexMap& m);
    std::string getName() const override { return "HJPS"; }
    std::vector<Hex> findPath(Hex start, Hex end) override;
    void reset() override;

    static const Hex INVALID_HEX;

private:
    static int wrapDir(int d) {
        d %= 6;
        if (d < 0) d += 6;
        return d;
    }

    // 快速将 q, r 转换为索引（内联优化）
    inline int getIndex(int q, int r) const {
        int col = q + (r - (r & 1)) / 2;
        if (col < 0 || col >= map.width || r < 0 || r >= map.height) {
            return -1;
        }
        return r * map.width + col;
    }

    // 快速检查是否可走（直接传 q, r，避免创建 Hex 对象）
    inline bool isWalkable(int q, int r) const {
        int idx = getIndex(q, r);
        return idx != -1 && map.isWalkableIdx(idx);
    }

    int  getDirectionIndex(Hex from, Hex to) const;
    bool hasForcedNeighbors(const Hex& curr, int dir) const;
    Hex  jump(const Hex& from, int dir, const Hex& end) const;

    void ensureCapacity();
    void initNode(int idx);
    Hex  cubeRound(float x, float y, float z) const;
    std::vector<Hex> reconstructPath(int endIdx) const;
};