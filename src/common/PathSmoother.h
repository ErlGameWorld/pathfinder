#pragma once
#include "../map/HexMap.h"
#include <vector>

class PathSmoother {
public:
    explicit PathSmoother(const HexMap& map);

    // Floyd smoothing (engineering-grade, obstacle-safe)
    std::vector<Hex> smooth(const std::vector<Hex>& path);

private:
    const HexMap& map;

    bool lineOfSight(const Hex& a, const Hex& b) const;
};
