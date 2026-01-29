#pragma once
#include "IFinder.h"
#include "../map/HexMap.h"
#include <vector>

class FlowField : public IFinder {
    HexMap& map;
    std::vector<int> parentMap; // Index -> Parent Index (towards End)
    Hex currentEnd;
    bool built = false;

public:
    FlowField(HexMap& m);
    std::string getName() const override { return "FlowField"; }
    std::vector<Hex> findPath(Hex start, Hex end) override;
    void reset() override;
    void onMapUpdate(const std::vector<Hex>& changedHexes) override;
};
