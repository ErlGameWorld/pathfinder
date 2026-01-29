#include "JPS.h"
#include "../common/VizMacros.h"
#include <algorithm>

const Hex JPS::INVALID_HEX = Hex(-99999, -99999);

JPS::JPS(HexMap& m) : map(m) {
    nodeData.resize((size_t)map.width * (size_t)map.height);
}

void JPS::reset() {
    size_t requiredSize = (size_t)map.width * (size_t)map.height;
    if (nodeData.size() != requiredSize) nodeData.resize(requiredSize);
    std::fill(nodeData.begin(), nodeData.end(), Node());
    currentGen = 0;
}

int JPS::getDirectionIndex(Hex from, Hex to) {
    int dq = to.q - from.q;
    int dr = to.r - from.r;

    if (dq == 0 && dr == 0) return -1;
    if (dr == 0) return (dq > 0) ? 0 : 3;
    if (dq == 0) return (dr > 0) ? 5 : 2;
    if (dq + dr == 0) return (dq > 0) ? 1 : 4;
    return -1; // 不在同一条射线上
}

bool JPS::hasForcedNeighbors(const Hex& curr, int dirIdx) {
    // 采用你最初推导的版本（对 axial 方向定义 HEX_DIRS 有效）：
    // 若 dir+2（左后）被挡 且 dir+1（左前）可走 => 强制邻居存在
    // 若 dir-2（右后）被挡 且 dir-1（右前）可走 => 强制邻居存在
    int dLB = wrapDir(dirIdx + 2);
    int dLF = wrapDir(dirIdx + 1);
    int dRB = wrapDir(dirIdx - 2);
    int dRF = wrapDir(dirIdx - 1);

    Hex leftBack  = curr + HEX_DIRS[dLB];
    Hex leftFore  = curr + HEX_DIRS[dLF];
    Hex rightBack = curr + HEX_DIRS[dRB];
    Hex rightFore = curr + HEX_DIRS[dRF];

    if (!map.isWalkable(leftBack)  && map.isWalkable(leftFore))  return true;
    if (!map.isWalkable(rightBack) && map.isWalkable(rightFore)) return true;
    return false;
}

Hex JPS::jump(const Hex& from, int dirIdx, const Hex& end) {
    const Hex dirVec = HEX_DIRS[dirIdx];

    // 第一步不可走：直接失败
    Hex first = from + dirVec;
    if (!map.isWalkable(first)) return INVALID_HEX;

    Hex prev = from;
    Hex cur  = from;

    while (true) {
        cur = cur + dirVec;

        // 撞墙/越界：prev 是走廊尽头，作为跳点返回（允许转弯）
        if (!map.isWalkable(cur)) return prev;

        if (cur == end) return cur;

        // 强制邻居：跳点
        if (hasForcedNeighbors(cur, dirIdx)) return cur;

        // ===== 关键补全：空地也能拐弯 =====
        // 若继续前进不再让你更接近目标，则在此处返回作为“拐弯跳点”
        Hex fwd = cur + dirVec;
        if (!map.isWalkable(fwd)) {
            // 前方下一步撞墙：cur 是尽头
            return cur;
        }
        int hCur = cur.distance(end);
        int hFwd = fwd.distance(end);
        if (hFwd >= hCur) {
            return cur;
        }
        // ===============================

        prev = cur;
    }
}

std::vector<Hex> JPS::findPath(Hex start, Hex end) {
    size_t requiredSize = (size_t)map.width * (size_t)map.height;
    if (nodeData.size() != requiredSize) {
        nodeData.resize(requiredSize);
        std::fill(nodeData.begin(), nodeData.end(), Node());
        currentGen = 0;
    }

    if (!map.isWalkable(start) || !map.isWalkable(end)) return {};
    if (start == end) return { start };

    currentGen++;
    if (currentGen == 0) { // 溢出保护
        std::fill(nodeData.begin(), nodeData.end(), Node());
        currentGen = 1;
    }

    const int startIdx = map.getIndex(start);
    const int endIdx   = map.getIndex(end);
    if (startIdx < 0 || endIdx < 0) return {};

    auto initNodeIfNewGen = [&](int idx) {
        if (nodeData[idx].visitedGen != currentGen) {
            nodeData[idx].visitedGen = currentGen;
            nodeData[idx].closedGen  = 0;
            nodeData[idx].gScore     = 4000000000000000000LL;
            nodeData[idx].parentIdx  = -1;
        }
    };

    initNodeIfNewGen(startIdx);
    nodeData[startIdx].gScore    = 0;
    nodeData[startIdx].parentIdx = -1;

    struct PQItem {
        int64_t f;
        int64_t g;
        int idx;
        bool operator>(const PQItem& o) const { return f > o.f; }
    };

    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> openSet;
    openSet.push({ (int64_t)start.distance(end) * 1000, 0, startIdx });

    while (!openSet.empty()) {
        PQItem it = openSet.top();
        openSet.pop();

        int currIdx = it.idx;

        if (nodeData[currIdx].visitedGen != currentGen) continue;
        if (it.g != nodeData[currIdx].gScore) continue;          // stale
        if (nodeData[currIdx].closedGen == currentGen) continue; // closed

        Hex curr = map.getHex(currIdx);

        nodeData[currIdx].closedGen = currentGen;

        VIZ_LOG(curr);

        if (currIdx == endIdx) {
            // 重建 jump 点链
            std::vector<Hex> jumps;
            for (int c = currIdx; c != -1; c = nodeData[c].parentIdx) {
                jumps.push_back(map.getHex(c));
            }
            std::reverse(jumps.begin(), jumps.end());

            // 填充每段射线
            std::vector<Hex> path;
            if (!jumps.empty()) path.push_back(jumps[0]);

            for (size_t i = 0; i + 1 < jumps.size(); ++i) {
                Hex p1 = jumps[i];
                Hex p2 = jumps[i + 1];
                int dist = p1.distance(p2);
                if (dist <= 0) continue;

                int dir = getDirectionIndex(p1, p2);
                if (dir == -1) {
                    // 正常不应发生（jump 只产生射线段）。保底：直接返回 jumps
                    return jumps;
                }

                for (int k = 0; k < dist; ++k) {
                    p1 = p1 + HEX_DIRS[dir];
                    path.push_back(p1);
                }
            }
            return path;
        }

        // 生成 successor 方向（bitmask 去重）
        uint8_t dirMask = 0;
        auto addDir = [&](int d) { dirMask |= (uint8_t)(1u << wrapDir(d)); };

        int pIdx = nodeData[currIdx].parentIdx;
        if (pIdx == -1) {
            dirMask = 0x3F; // 起点：6 个方向
        } else {
            Hex parent = map.getHex(pIdx);
            int inDir = getDirectionIndex(parent, curr);
            if (inDir == -1) {
                dirMask = 0x3F;
            } else {
                // Hex 的“前进锥”：直行 + 左右 60°（保证空地可转向）
                addDir(inDir);
                addDir(inDir + 1);
                addDir(inDir - 1);
            }
        }

        // 执行跳跃扩展
        for (int dir = 0; dir < 6; ++dir) {
            if (!(dirMask & (1u << dir))) continue;

            // 如果第一步就不可走，跳过
            Hex n1 = curr + HEX_DIRS[dir];
            if (!map.isWalkable(n1)) continue;

            Hex jp = jump(curr, dir, end);
            if (jp == INVALID_HEX) continue;

            int jpIdx = map.getIndex(jp);
            if (jpIdx < 0) continue;

            initNodeIfNewGen(jpIdx);

            int64_t newG = nodeData[currIdx].gScore + (int64_t)curr.distance(jp) * 1000;
            if (newG < nodeData[jpIdx].gScore) {
                nodeData[jpIdx].gScore    = newG;
                nodeData[jpIdx].parentIdx = currIdx;

                int64_t h = (int64_t)jp.distance(end) * 1000;
                openSet.push({ newG + h, newG, jpIdx });
            }
        }
    }

    return {};
}