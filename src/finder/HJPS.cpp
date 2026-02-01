#include "HJPS.h"
#include "../common/VizMacros.h"
#include <algorithm>
#include <cmath>

const Hex HJPS::INVALID_HEX = Hex(-99999, -99999);

static constexpr int64_t COST_UNIT = 1000LL;
static constexpr int64_t INF_COST = INT64_MAX / 2;

// ============================================================
// 构造与初始化
// ============================================================
HJPS::HJPS(HexMap& m) : map(m) {
ensureCapacity();
}

void HJPS::ensureCapacity() {
size_t n = static_cast<size_t>(map.width) * map.height;
if (nodes.size() != n) {
nodes.resize(n);
currentGen = 0;
}
}

void HJPS::reset() {
ensureCapacity();
std::fill(nodes.begin(), nodes.end(), Node());
currentGen = 0;
}

void HJPS::initNode(int idx) {
if (nodes[idx].visitedGen != currentGen) {
nodes[idx].visitedGen = currentGen;
nodes[idx].closedGen = 0;
nodes[idx].gScore = INF_COST;
nodes[idx].parentIdx = -1;
nodes[idx].parentDir = -1;
}
}

// ============================================================
// 方向工具
// ============================================================
int HJPS::getDirectionIndex(Hex from, Hex to) const {
int dq = to.q - from.q;
int dr = to.r - from.r;

    if (dq == 0 && dr == 0) return -1;

    // 六边形6个方向:
    // 0: (+1,  0)  1: (+1, -1)  2: (0, -1)
    // 3: (-1,  0)  4: (-1, +1)  5: (0, +1)
    if (dr == 0) return (dq > 0) ? 0 : 3;
    if (dq == 0) return (dr > 0) ? 5 : 2;
    if (dq + dr == 0) return (dq > 0) ? 1 : 4;

    return -1;
}

// ============================================================
// 强制邻居检测 - 优化版：直接传 q, r，避免创建 Hex 对象
// ============================================================
bool HJPS::hasForcedNeighbors(const Hex& curr, int dir) const {
/*
*        [dir+1]         [dir-1]
*         左前             右前
*            \           /
*             \         /
*    [dir+2]    curr    [dir-2]
*     左后       ↑        右后
*              [dir]
*/

    int dLB = wrapDir(dir + 2);
    int dLF = wrapDir(dir + 1);
    int dRB = wrapDir(dir - 2);
    int dRF = wrapDir(dir - 1);

    // 直接计算邻居坐标
    const Hex& dirLB = HEX_DIRS[dLB];
    const Hex& dirLF = HEX_DIRS[dLF];
    const Hex& dirRB = HEX_DIRS[dRB];
    const Hex& dirRF = HEX_DIRS[dRF];

    // 左后不可走且左前可走 → 有强制邻居
    if (!isWalkable(curr.q + dirLB.q, curr.r + dirLB.r) && 
        isWalkable(curr.q + dirLF.q, curr.r + dirLF.r)) {
        return true;
    }
    
    // 右后不可走且右前可走 → 有强制邻居
    if (!isWalkable(curr.q + dirRB.q, curr.r + dirRB.r) && 
        isWalkable(curr.q + dirRF.q, curr.r + dirRF.r)) {
        return true;
    }

    return false;
}

// ============================================================
// 核心跳跃函数 - 优化版：使用 isWalkable(q, r) 避免 Hex 对象
// ============================================================
Hex HJPS::jump(const Hex& from, int dir, const Hex& end) const {
const Hex& dv = HEX_DIRS[dir];
int curQ = from.q + dv.q;
int curR = from.r + dv.r;

    // 第一步不可走
    if (!isWalkable(curQ, curR)) {
        return INVALID_HEX;
    }

    // 侧向方向向量
    int dLeft  = wrapDir(dir + 1);
    int dRight = wrapDir(dir - 1);
    const Hex& leftVec  = HEX_DIRS[dLeft];
    const Hex& rightVec = HEX_DIRS[dRight];

    // 预计算方向向量分量（避免重复访问）
    const int dvQ = dv.q, dvR = dv.r;
    const int leftQ = leftVec.q, leftR = leftVec.r;
    const int rightQ = rightVec.q, rightR = rightVec.r;
    
    // 终点的第三坐标 s = -q - r
    const int endS = -end.q - end.r;
    const int dirS = -dvQ - dvR;
    const int endQ = end.q, endR = end.r;

    while (true) {
        // === 条件1: 到达终点 ===
        if (curQ == endQ && curR == endR) {
            return Hex(curQ, curR);
        }

        // === 条件2: 有强制邻居（内联优化）===
        // 左后不可走且左前可走 → 有强制邻居
        int dLB = wrapDir(dir + 2);
        int dLF = wrapDir(dir + 1);
        int dRB = wrapDir(dir - 2);
        int dRF = wrapDir(dir - 1);
        const Hex& dirLB = HEX_DIRS[dLB];
        const Hex& dirLF = HEX_DIRS[dLF];
        const Hex& dirRB = HEX_DIRS[dRB];
        const Hex& dirRF = HEX_DIRS[dRF];
        
        if ((!isWalkable(curQ + dirLB.q, curR + dirLB.r) && 
             isWalkable(curQ + dirLF.q, curR + dirLF.r)) ||
            (!isWalkable(curQ + dirRB.q, curR + dirRB.r) && 
             isWalkable(curQ + dirRF.q, curR + dirRF.r))) {
            return Hex(curQ, curR);
        }

        // === 条件3: 终点在侧向射线上（简化判断）===
        int dq = endQ - curQ;
        int dr = endR - curR;

        // 检查左前方向
        if (leftQ == 0) {
            if (dq == 0 && dr * leftR > 0) return Hex(curQ, curR);
        } else if (leftR == 0) {
            if (dr == 0 && dq * leftQ > 0) return Hex(curQ, curR);
        } else {
            if (dq + dr == 0 && dq * leftQ > 0) return Hex(curQ, curR);
        }

        // 检查右前方向
        if (rightQ == 0) {
            if (dq == 0 && dr * rightR > 0) return Hex(curQ, curR);
        } else if (rightR == 0) {
            if (dr == 0 && dq * rightQ > 0) return Hex(curQ, curR);
        } else {
            if (dq + dr == 0 && dq * rightQ > 0) return Hex(curQ, curR);
        }

        // === 条件4: 坐标轴对齐（终点在正前方）===
        int curS = -curQ - curR;
        if (dvQ != 0 && curQ == endQ) return Hex(curQ, curR);
        if (dvR != 0 && curR == endR) return Hex(curQ, curR);
        if (dirS != 0 && curS == endS) return Hex(curQ, curR);

        // === 检查下一步 ===
        int nextQ = curQ + dvQ;
        int nextR = curR + dvR;

        if (!isWalkable(nextQ, nextR)) {
            // 撞墙时的处理：检查侧向是否可走
            if (isWalkable(curQ + leftQ, curR + leftR) || 
                isWalkable(curQ + rightQ, curR + rightR)) {
                return Hex(curQ, curR);
            }
            return INVALID_HEX;
        }

        curQ = nextQ;
        curR = nextR;
    }
}

// ============================================================
// Cube Round
// ============================================================
Hex HJPS::cubeRound(float x, float y, float z) const {
int rx = static_cast<int>(std::round(x));
int ry = static_cast<int>(std::round(y));
int rz = static_cast<int>(std::round(z));

    float xd = std::abs(rx - x);
    float yd = std::abs(ry - y);
    float zd = std::abs(rz - z);

    if (xd > yd && xd > zd) {
        rx = -ry - rz;
    } else if (yd > zd) {
        ry = -rx - rz;
    }

    return Hex(rx, ry);
}

// ============================================================
// 路径重建
// ============================================================
std::vector<Hex> HJPS::reconstructPath(int endIdx) const {
std::vector<int> indices;
for (int c = endIdx; c != -1; c = nodes[c].parentIdx) {
indices.push_back(c);
}
std::reverse(indices.begin(), indices.end());

    if (indices.empty()) return {};

    std::vector<Hex> path;
    path.push_back(map.getHex(indices[0]));

    for (size_t i = 0; i + 1 < indices.size(); ++i) {
        Hex p1 = map.getHex(indices[i]);
        Hex p2 = map.getHex(indices[i + 1]);

        int dir = getDirectionIndex(p1, p2);
        int dist = p1.distance(p2);

        if (dir != -1) {
            const Hex& dv = HEX_DIRS[dir];
            Hex cur = p1;
            for (int k = 0; k < dist; ++k) {
                cur = cur + dv;
                path.push_back(cur);
            }
        } else {
            // 非主轴：插值
            for (int k = 1; k <= dist; ++k) {
                float t = static_cast<float>(k) / dist;
                float fx = p1.q + (p2.q - p1.q) * t;
                float fy = p1.r + (p2.r - p1.r) * t;
                float fz = (-p1.q - p1.r) + ((-p2.q - p2.r) - (-p1.q - p1.r)) * t;
                path.push_back(cubeRound(fx, fy, fz));
            }
        }
    }

    return path;
}

// ============================================================
// 主搜索函数
// ============================================================
std::vector<Hex> HJPS::findPath(Hex start, Hex end) {
ensureCapacity();

    if (!map.isWalkable(start) || !map.isWalkable(end)) {
        return {};
    }
    if (start == end) {
        return { start };
    }

    ++currentGen;
    if (currentGen == 0) {
        std::fill(nodes.begin(), nodes.end(), Node());
        currentGen = 1;
    }

    const int startIdx = map.getIndex(start);
    const int endIdx = map.getIndex(end);
    if (startIdx < 0 || endIdx < 0) return {};

    initNode(startIdx);
    nodes[startIdx].gScore = 0;
    nodes[startIdx].parentIdx = -1;
    nodes[startIdx].parentDir = -1;

    struct PQItem {
        int64_t f;
        int64_t g;
        int idx;
        bool operator>(const PQItem& o) const { return f > o.f; }
    };

    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> open;
    open.push({ start.distance(end) * COST_UNIT, 0, startIdx });

    while (!open.empty()) {
        PQItem item = open.top();
        open.pop();

        int currIdx = item.idx;

        if (nodes[currIdx].visitedGen != currentGen) continue;
        if (item.g != nodes[currIdx].gScore) continue;
        if (nodes[currIdx].closedGen == currentGen) continue;

        nodes[currIdx].closedGen = currentGen;
        Hex curr = map.getHex(currIdx);

        VIZ_LOG(curr);

        if (currIdx == endIdx) {
            return reconstructPath(endIdx);
        }

        // === 确定搜索方向 ===
        uint8_t dirMask = 0;
        int8_t inDir = nodes[currIdx].parentDir;

        if (inDir < 0) {
            // 起点：搜索所有方向
            dirMask = 0x3F;
        } else {
            // JPS方向剪枝：前方 + 左前 + 右前
            dirMask |= (1u << inDir);
            dirMask |= (1u << wrapDir(inDir + 1));
            dirMask |= (1u << wrapDir(inDir - 1));

            // 【重要】检查强制邻居产生的额外方向
            // 如果左后被阻挡，左前方向需要单独探索（已包含在上面）
            // 如果右后被阻挡，右前方向需要单独探索（已包含在上面）
        }

        // === 探索各方向 ===
        for (int dir = 0; dir < 6; ++dir) {
            if (!(dirMask & (1u << dir))) continue;

            Hex jp = jump(curr, dir, end);
            if (jp == INVALID_HEX) continue;

            int jpIdx = map.getIndex(jp);
            if (jpIdx < 0) continue;

            initNode(jpIdx);

            int64_t stepCost = static_cast<int64_t>(curr.distance(jp)) * COST_UNIT;
            int64_t newG = nodes[currIdx].gScore + stepCost;

            if (newG < nodes[jpIdx].gScore) {
                nodes[jpIdx].gScore = newG;
                nodes[jpIdx].parentIdx = currIdx;
                nodes[jpIdx].parentDir = static_cast<int8_t>(dir);

                int64_t h = static_cast<int64_t>(jp.distance(end)) * COST_UNIT;
                open.push({ newG + h, newG, jpIdx });
            }
        }
    }

    return {};
}