#pragma once
#include "IFinder.h"
#include "../map/HexMap.h"
#include <vector>
#include <string>

class BFS : public IFinder {
private:
    HexMap& map;

    struct Node {
        int visitedGen = 0; 
        int parentIdx = -1;
    };

    std::vector<Node> nodeData;
    
    // 【优化3】手动模拟队列，避免 std::queue 的动态分配
    std::vector<int> queueData; 
    
    int currentGen = 0;

    // 包装器：尽量使用 map 的 inline 方法
    inline int getIdx(const Hex& h) const {
        return map.getIndex(h); 
    }

    inline Hex getHex(int idx) const {
        return map.getHex(idx);
    }

public:
    BFS(HexMap& m);
    virtual ~BFS() = default;

    std::string getName() const override { return "BFS"; }
    std::vector<Hex> findPath(Hex start, Hex end) override;
    void reset() override;
};