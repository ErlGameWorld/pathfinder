#pragma once
#include <vector>
#include <string>
#include "../common/Hex.h"

class IFinder {
public:
    virtual ~IFinder() = default;
    
    // Core pathfinding function
    virtual std::vector<Hex> findPath(Hex start, Hex end) = 0;
    
    // Get algorithm name
    virtual std::string getName() const = 0;
    
    // For algorithms that need to rebuild internal structures when map changes
    virtual void onMapUpdate(const std::vector<Hex>& changedHexes) {}
    
    // For algorithms that need to rebuild everything (e.g. map resize)
    virtual void reset() {}
    
    // For algorithms that have a build step (like DHPA*)
    virtual void buildGraph() {}
};
