#include "BiAStar.h"
#include "../common/VizMacros.h"
#include <algorithm>
#include <cmath>
#include <limits>

// 定义方向常量，增加代码可读性
const uint8_t MASK_FWD = 1; // 0001
const uint8_t MASK_BWD = 2; // 0010

BiAStar::BiAStar(HexMap& m) : map(m) {
    nodeData.resize(map.width * map.height);
}

void BiAStar::reset() {
    int targetSize = map.width * map.height;
    // 只有当地图尺寸变化时才重新分配内存
    if (nodeData.size() != targetSize) {
        nodeData.clear();
        nodeData.resize(targetSize);
        currentGen = 0;
    }
}

std::vector<Hex> BiAStar::findPath(Hex start, Hex end) {
    // 1. 基础合法性检查
    // isWalkable 内部已包含边界检查
    if (!map.isWalkable(start) || !map.isWalkable(end)) return {};
    if (start == end) return {start};

    // 2. Generation 管理 (免 memset 优化)
    if (currentGen == std::numeric_limits<uint32_t>::max()) {
        currentGen = 0;
        // 只有溢出时才清零，利用 CPU 的 rep stosd 指令，速度很快
        std::fill(nodeData.begin(), nodeData.end(), Node());
    }
    currentGen++;

    int startIdx = getIdx(start);
    int endIdx = getIdx(end);
    if (startIdx == -1 || endIdx == -1) return {};

    // 3. 初始化起点与终点数据
    // 起点
    Node& startNode = nodeData[startIdx];
    startNode.visitedGen = currentGen;
    startNode.visitMask = MASK_FWD;
    startNode.gScoreFwd = 0;
    startNode.gScoreBwd = 0xFFFFFFFF;
    startNode.parentFwd = -1;

    // 终点
    Node& endNode = nodeData[endIdx];
    // 如果终点恰好是脏数据（上次寻路的残留），需要初始化
    if (endNode.visitedGen != currentGen) {
        endNode.visitedGen = currentGen;
        endNode.visitMask = 0;
        endNode.gScoreFwd = 0xFFFFFFFF;
    }
    endNode.visitMask |= MASK_BWD;
    endNode.gScoreBwd = 0;
    endNode.parentBwd = -1;

    // 4. 准备优先队列
    typedef std::pair<unsigned int, int> PQNode; // <F_Score, Index>
    // 使用 vector 作为底层容器比 deque 略快
    std::priority_queue<PQNode, std::vector<PQNode>, std::greater<PQNode>> openFwd;
    std::priority_queue<PQNode, std::vector<PQNode>, std::greater<PQNode>> openBwd;

    // 启发式函数
    auto heuristic = [&](int idx, const Hex& target) -> unsigned int {
        // 【优化】乘以 1001 而不是 1000 用于 Tie-Breaking (打破平局)
        // 这会让 A* 在代价相同的路径中优先选择更靠近终点的，减少节点扩展
        return (unsigned int)getHex(idx).distance(target) * 1001;
    };

    openFwd.push({heuristic(startIdx, end), startIdx});
    openBwd.push({heuristic(endIdx, start), endIdx});

    int meetIdx = -1;
    unsigned int bestPathLen = 0xFFFFFFFF;

    // -------------------------------------------------------------------------
    // 核心扩展逻辑 (Lambda 封装)
    // -------------------------------------------------------------------------
    auto expand = [&](std::priority_queue<PQNode, std::vector<PQNode>, std::greater<PQNode>>& openSet,
                      bool isForward, const Hex& targetHex) -> bool {

        int uIdx = openSet.top().second;
        unsigned int uF = openSet.top().first;
        openSet.pop();

        Node& uNode = nodeData[uIdx];
        unsigned int uG = isForward ? uNode.gScoreFwd : uNode.gScoreBwd;

        // Lazy Removal: 如果取出的 F 值比当前的优值要差，说明是旧数据，跳过
        // 这里做一个简单的剪枝：如果 uF 已经比当前找到的最优路径长，就不需要扩展了
        if (uF >= bestPathLen) return false;

        // 检查相遇
        uint8_t targetMask = isForward ? MASK_BWD : MASK_FWD;
        if (uNode.visitMask & targetMask) {
            unsigned int total = uNode.gScoreFwd + uNode.gScoreBwd;
            if (total < bestPathLen) {
                bestPathLen = total;
                meetIdx = uIdx;
                return true; // 找到了连接点
            }
        }

        Hex uHex = getHex(uIdx);
        // 可视化调试 (仅在 Debug 模式下开启)
        VIZ_LOG(uHex);

        for (const auto& dir : HEX_DIRS) {
            Hex vHex = uHex + dir;
            int vIdx = getIdx(vHex);

            if (vIdx == -1) continue;
            // 【性能关键】利用 HexMap 的极速接口查表
            if (!map.isWalkableIdx(vIdx)) continue;

            Node& vNode = nodeData[vIdx];

            // 惰性初始化 (Lazy Init)
            if (vNode.visitedGen != currentGen) {
                vNode.visitedGen = currentGen;
                vNode.visitMask = 0;
                vNode.gScoreFwd = 0xFFFFFFFF;
                vNode.gScoreBwd = 0xFFFFFFFF;
            }

            unsigned int cost = 1000; // 移动代价
            unsigned int newG = uG + cost;

            if (isForward) {
                if (newG < vNode.gScoreFwd) {
                    vNode.gScoreFwd = newG;
                    vNode.parentFwd = uIdx;
                    vNode.visitMask |= MASK_FWD;
                    openSet.push({newG + heuristic(vIdx, end), vIdx});
                }
            } else {
                if (newG < vNode.gScoreBwd) {
                    vNode.gScoreBwd = newG;
                    vNode.parentBwd = uIdx;
                    vNode.visitMask |= MASK_BWD;
                    openSet.push({newG + heuristic(vIdx, start), vIdx});
                }
            }
        }
        return false;
    };

    // -------------------------------------------------------------------------
    // 搜索循环 (平衡双向扩展)
    // -------------------------------------------------------------------------
    while (!openFwd.empty() && !openBwd.empty()) {

        // 终止条件：一旦找到任一相遇点，立即退出 (追求速度)
        // 如果追求绝对最短路径，可将此判断改为 if (minF_fwd + minF_bwd >= bestPathLen)
        if (bestPathLen != 0xFFFFFFFF) {
             break;
        }

        // 平衡扩展策略：总是扩展节点较少的那一端
        // 这在“血管状”或迷宫地图中极其有效，避免一方扩散过大
        if (openFwd.size() < openBwd.size()) {
            // 如果找到了路径，立即 break
            if (expand(openFwd, true, end)) break;
        } else {
            // 如果找到了路径，立即 break
            if (expand(openBwd, false, start)) break;
        }
    }

    // 5. 路径重建
    if (meetIdx != -1) {
        std::vector<Hex> path;
        // 预估路径长度，减少 vector 扩容开销
        path.reserve(128);

        // A. 正向部分 (Start -> ... -> Meet)
        int curr = meetIdx;
        while (curr != -1) {
            path.push_back(getHex(curr));
            curr = nodeData[curr].parentFwd;
        }
        std::reverse(path.begin(), path.end());

        // B. 反向部分 (Meet -> ... -> End)
        // 注意：Meet 点已经在上面加过了，所以从 Meet 的父节点开始
        curr = nodeData[meetIdx].parentBwd;
        while (curr != -1) {
            path.push_back(getHex(curr));
            curr = nodeData[curr].parentBwd;
        }

        return path;
    }

    return {};
}