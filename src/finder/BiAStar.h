#pragma once
#include "IFinder.h"
#include "../map/HexMap.h"
#include <vector>
#include <queue>
#include <string>

class BiAStar : public IFinder {
    HexMap& map;

    struct Node {
        uint32_t visitedGen = 0;
        // 0=None, 1=FwdOpen, 2=FwdClosed, 4=BwdOpen, 8=BwdClosed (位掩码)
        uint8_t visitMask = 0;

        // 代价评分，使用 unsigned int 足够应对大多数地图
        unsigned int gScoreFwd = 0xFFFFFFFF;
        unsigned int gScoreBwd = 0xFFFFFFFF;
        int parentFwd = -1;
        int parentBwd = -1;
    };

    // 内存池：重复利用，避免每次寻路重新分配内存
    std::vector<Node> nodeData;
    uint32_t currentGen = 0;

    // 内联辅助函数
    inline int getIdx(const Hex& h) const { return map.getIndex(h); }
    inline Hex getHex(int idx) const { return map.getHex(idx); }

public:
    BiAStar(HexMap& m);
    virtual ~BiAStar() = default;

    std::string getName() const override { return "BiAStar"; }

    std::vector<Hex> findPath(Hex start, Hex end) override;

    void reset() override;
};