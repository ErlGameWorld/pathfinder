#pragma once
#include "IFinder.h"
#include "../map/HexMap.h"
#include <vector>
#include <queue>
#include <cstdint>
#include <string>

class JPS : public IFinder {
    HexMap& map;

    struct Node {
        uint32_t visitedGen = 0;
        uint32_t closedGen  = 0;
        int64_t  gScore     = 4000000000000000000LL;
        int      parentIdx  = -1;
    };

    std::vector<Node> nodeData;
    uint32_t currentGen = 0;

public:
    explicit JPS(HexMap& m);
    std::string getName() const override { return "JPS"; }
    std::vector<Hex> findPath(Hex start, Hex end) override;
    void reset() override;

    static const Hex INVALID_HEX;

private:
    static int wrapDir(int d) { d %= 6; if (d < 0) d += 6; return d; }

    int  getDirectionIndex(Hex from, Hex to);

    // travelDir 是扫描方向：若在 curr 处出现 forced neighbor 布局，则 curr 是跳点
    bool hasForcedNeighbors(const Hex& curr, int travelDir);

    // 直线扫描跳跃（无递归）
    Hex  jump(const Hex& from, int travelDir, const Hex& end);
};