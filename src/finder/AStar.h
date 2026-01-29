#pragma once

#include "../map/HexMap.h"
#include "IFinder.h"
#include <vector>
#include <string>
#include <cstdint>

class AStar : public IFinder {
private:
    const HexMap& map;

    struct NodeData {
        uint32_t visitedGen = 0;
        int64_t gScore = 0;
        int parentIdx = -1;
    };

    std::vector<NodeData> nodeData;
    uint32_t currentGen = 0;

    struct SearchNode {
        int64_t fScore;
        int64_t gScore;
        int64_t tie;
        int index;
    };

    struct NodeComparator {
        bool operator()(const SearchNode& a, const SearchNode& b) const {
            if (a.fScore != b.fScore) {
                return a.fScore > b.fScore;
            }
            return a.tie > b.tie;
        }
    };

    std::vector<SearchNode> openSetContainer;

    // 内联 helper：将 Hex 转为 Offset Coordinates (Odd-R)
    inline void hexToOffset(const Hex& h, int& col, int& row) const {
        col = h.q + (h.r - (h.r & 1)) / 2;
        row = h.r;
    }

    // 索引计算包装
    inline int getMapIndex(int col, int row) const {
        if (col < 0 || col >= map.width || row < 0 || row >= map.height) return -1;
        return row * map.width + col;
    }

public:
    static const int64_t STEP_COST = 10;

    explicit AStar(const HexMap& m);

    std::string getName() const override { return "AStar"; }
    std::vector<Hex> findPath(Hex start, Hex end) override;
    void reset() override;

    int64_t findPathInternal(Hex start, Hex end, std::vector<Hex>* outPath = nullptr, int clusterId = -1, int maxSteps = -1);
};