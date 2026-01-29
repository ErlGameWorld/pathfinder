#include "BFS.h"
#include "../common/VizMacros.h"
#include <algorithm> // for reverse, abs
#include <limits>    // for numeric_limits

BFS::BFS(HexMap& m) : map(m) {
    int size = map.width * map.height;
    nodeData.resize(size);
    // 预分配最大容量，确保在 findPath 中 push_back 永不触发扩容
    queueData.reserve(size);
}

void BFS::reset() {
    int targetSize = map.width * map.height;
    
    // 只有地图尺寸变化时才重新分配内存
    // 正常关卡切换或重置不需要跑这个分支
    if (nodeData.size() != targetSize) {
        nodeData.clear();
        nodeData.resize(targetSize);
        
        queueData.clear();
        queueData.reserve(targetSize); 
        
        currentGen = 0; 
    } 
    // 保持 currentGen 不变，利用 visitedGen 机制避免 O(N) 清零
}

std::vector<Hex> BFS::findPath(Hex start, Hex end) {
    // 1. 基础合法性检查
    // isWalkable 内部有边界检查
    if (!map.isWalkable(start) || !map.isWalkable(end)) return {};
    if (start == end) return {start};

    // 2. 代数管理 (Generation System)
    // 防止 currentGen 溢出 (约 20 亿次寻路后才会触发一次)
    if (currentGen >= std::numeric_limits<int>::max() - 1) {
        currentGen = 0;
        // 只有溢出时才执行昂贵的清零操作
        std::fill(nodeData.begin(), nodeData.end(), Node{0, -1});
    }
    currentGen++; 

    // 获取起点终点索引
    int startIdx = getIdx(start);
    int endIdx = getIdx(end);
    
    // 二次确认索引有效性
    if (startIdx == -1 || endIdx == -1) return {};

    // 3. 初始化手动队列 (Zero Allocation)
    queueData.clear(); // size 置 0，但 capacity 保持不变，无内存释放
    
    // 使用 size_t 避免与 vector.size() 比较时的符号警告
    size_t qHead = 0; 

    // 起点入队
    nodeData[startIdx].visitedGen = currentGen;
    nodeData[startIdx].parentIdx = -1;
    queueData.push_back(startIdx);

    bool found = false;

    // 4. 核心搜索循环
    // qHead < queueData.size() 模拟 !queue.empty()
    while(qHead < queueData.size()) {
        int currIdx = queueData[qHead++]; 

        // 调试宏：请确保在 Release 模式下该宏为空，否则会严重拖慢循环
        // 如果 VIZ_LOG 需要 Hex 对象，这里才转换，否则可省略
        VIZ_LOG(getHex(currIdx)); 

        // 获取当前 Hex 用于计算邻居 (这一步无法避免，需要几何关系)
        Hex currHex = getHex(currIdx);

        for (const auto& dir : HEX_DIRS) {
            Hex next = currHex + dir;
            int nextIdx = getIdx(next);

            // A. 边界检查
            if (nextIdx == -1) continue;

            // B. 访问状态检查 (最快：读内存)
            if (nodeData[nextIdx].visitedGen == currentGen) continue;

            // C. 地形检查 (次快：读地图数据)
            // 必须使用 isWalkableIdx 以避免重复的 Hex->Index 计算
            if (!map.isWalkableIdx(nextIdx)) continue; 

            // 记录路径信息
            nodeData[nextIdx].parentIdx = currIdx;
            nodeData[nextIdx].visitedGen = currentGen;

            // 【优化】提前退出 (Early Exit)
            // 在入队前检查终点，省去一次入队出队开销
            if (nextIdx == endIdx) {
                found = true;
                goto EndSearch; 
            }

            queueData.push_back(nextIdx);
        }
    }

EndSearch:
    // 5. 路径回溯
    if (found) {
        std::vector<Hex> path;
        
        // 启发式预估内存：Hex 距离 + 少量缓冲
        // dist = (|dq| + |dr| + |ds|) / 2
        int dq = start.q - end.q;
        int dr = start.r - end.r;
        int ds = -dq - dr;
        int dist = (std::abs(dq) + std::abs(dr) + std::abs(ds)) / 2;
        path.reserve(dist + 8); 
        
        int curr = endIdx;
        while (curr != -1) {
            path.push_back(getHex(curr));
            curr = nodeData[curr].parentIdx;
        }
        
        std::reverse(path.begin(), path.end());
        return path;
    }

    return {};
}