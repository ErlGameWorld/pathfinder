#pragma once
#include "IFinder.h"
#include "../map/HexMap.h"
#include <vector>
#include <queue>
#include <cstdint>
#include <string>

class HJPS : public IFinder {
public:
    explicit HJPS(HexMap& m);

    std::vector<Hex> findPath(Hex start, Hex end) override;
    std::string getName() const override { return "HJPS"; }
    void reset() override;

private:
    HexMap& map;

    struct Node {
        uint32_t visitedGen = 0;
        int64_t gScore = INT64_MAX;
        int parentIdx = -1;
    };

    std::vector<Node> nodeData;
    uint32_t gen = 1;

    static const Hex INVALID_HEX;

    // ------------ Core helpers ------------
    int index(const Hex& h) const { return h.r * map.width + h.q; }
    bool inBounds(const Hex& h) const { return map.inBounds(h); }
    bool passable(const Hex& h) const { return map.isWalkable(h); }

    void initNodeIfNewGen(int idx);

    static int wrapDir(int d) { d %= 6; if (d < 0) d += 6; return d; }
    int getDirectionIndex(Hex from, Hex to);

    // ------------ HJPS specific ------------
    bool hasForcedNeighbors(const Hex& curr, int travelDir);
    bool hasIntermediateJump(const Hex& curr, int travelDir, const Hex& end);

    Hex  jump(const Hex& from, int travelDir, const Hex& end);
};
