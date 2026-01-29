#include "HJPS.h"
#include "../common/VizMacros.h"
#include <algorithm>
#include <limits>

using namespace std;

const Hex HJPS::INVALID_HEX = Hex(-99999, -99999);

// 六方向（与项目 Hex::neighbor(d) 保持一致）
static const int DIRS[6][2] = {
    { 1,  0}, // E
    { 1, -1}, // NE
    { 0, -1}, // NW
    {-1,  0}, // W
    {-1,  1}, // SW
    { 0,  1}  // SE
};

HJPS::HJPS(HexMap& m) : map(m) {
    nodeData.resize((size_t)map.width * (size_t)map.height);
}

void HJPS::reset() {
    size_t requiredSize = (size_t)map.width * (size_t)map.height;
    if (nodeData.size() != requiredSize)
        nodeData.resize(requiredSize);
    gen++;
    if (gen == 0) gen = 1;
}

void HJPS::initNodeIfNewGen(int idx) {
    if (nodeData[idx].visitedGen != gen) {
        nodeData[idx].visitedGen = gen;
        nodeData[idx].gScore = INT64_MAX;
        nodeData[idx].parentIdx = -1;
    }
}

int HJPS::getDirectionIndex(Hex from, Hex to) {
    int dq = to.q - from.q;
    int dr = to.r - from.r;
    for (int i = 0; i < 6; i++) {
        if (dq == DIRS[i][0] && dr == DIRS[i][1]) return i;
    }
    return -1;
}

// =======================================================
// 强制邻居判断（论文级）
// =======================================================
bool HJPS::hasForcedNeighbors(const Hex& curr, int travelDir) {
    int left = wrapDir(travelDir - 1);
    int right = wrapDir(travelDir + 1);

    Hex leftBlock(curr.q + DIRS[left][0], curr.r + DIRS[left][1]);
    Hex rightBlock(curr.q + DIRS[right][0], curr.r + DIRS[right][1]);

    Hex leftDiag(curr.q + DIRS[left][0] + DIRS[travelDir][0],
                 curr.r + DIRS[left][1] + DIRS[travelDir][1]);

    Hex rightDiag(curr.q + DIRS[right][0] + DIRS[travelDir][0],
                  curr.r + DIRS[right][1] + DIRS[travelDir][1]);

    if (inBounds(leftBlock) && !passable(leftBlock) &&
        inBounds(leftDiag) && passable(leftDiag))
        return true;

    if (inBounds(rightBlock) && !passable(rightBlock) &&
        inBounds(rightDiag) && passable(rightDiag))
        return true;

    return false;
}

// =======================================================
// 方向Ⅱ中间跳点检测（HJPS 核心创新）
// =======================================================
bool HJPS::hasIntermediateJump(const Hex& curr, int travelDir, const Hex& end) {
    // 左右分支方向
    int left = wrapDir(travelDir - 1);
    int right = wrapDir(travelDir + 1);

    Hex l = curr.neighbor(left);
    if (inBounds(l) && passable(l)) {
        Hex j = jump(curr, left, end);
        if (j != INVALID_HEX) return true;
    }

    Hex r = curr.neighbor(right);
    if (inBounds(r) && passable(r)) {
        Hex j = jump(curr, right, end);
        if (j != INVALID_HEX) return true;
    }

    return false;
}

// =======================================================
// Jump 主逻辑（论文级）
// =======================================================
Hex HJPS::jump(const Hex& from, int travelDir, const Hex& end) {
    Hex curr = from;

    while (true) {
        curr = curr.neighbor(travelDir);
        if (!inBounds(curr) || !passable(curr)) return INVALID_HEX;
        if (curr == end) return curr;

        // 强制邻居 ⇒ 跳点
        if (hasForcedNeighbors(curr, travelDir)) return curr;

        // 方向Ⅱ：中间跳点传播机制
        if (hasIntermediateJump(curr, travelDir, end)) return curr;
    }
}

// =======================================================
// 主搜索（A* + HJPS）
// =======================================================
std::vector<Hex> HJPS::findPath(Hex start, Hex end) {
    reset();

    struct PQItem {
        int64_t f, g;
        int idx;
        bool operator<(const PQItem& o) const { return f > o.f; }
    };

    priority_queue<PQItem> openSet;

    int startIdx = index(start);
    initNodeIfNewGen(startIdx);
    nodeData[startIdx].gScore = 0;
    openSet.push({ (int64_t)start.distance(end) * 1000, 0, startIdx });

    while (!openSet.empty()) {
        auto [f, g, currIdx] = openSet.top();
        openSet.pop();

        Hex curr(currIdx % map.width, currIdx / map.width);
        if (curr == end) break;

        int parentIdx = nodeData[currIdx].parentIdx;
        int parentDir = -1;
        if (parentIdx >= 0) {
            Hex parent(parentIdx % map.width, parentIdx / map.width);
            parentDir = getDirectionIndex(parent, curr);
        }

        // ========== 论文级邻居剪枝规则 ==========
        vector<int> scanDirs;
        if (parentDir < 0) {
            for (int d = 0; d < 6; d++) scanDirs.push_back(d);
        } else {
            scanDirs.push_back(parentDir);
            scanDirs.push_back(wrapDir(parentDir - 1));
            scanDirs.push_back(wrapDir(parentDir + 1));
        }

        for (int d : scanDirs) {
            Hex jp = jump(curr, d, end);
            if (jp == INVALID_HEX) continue;

            int jpIdx = index(jp);
            initNodeIfNewGen(jpIdx);

            int64_t newG = nodeData[currIdx].gScore +
                           (int64_t)curr.distance(jp) * 1000;

            if (newG < nodeData[jpIdx].gScore) {
                nodeData[jpIdx].gScore = newG;
                nodeData[jpIdx].parentIdx = currIdx;
                int64_t h = (int64_t)jp.distance(end) * 1000;
                openSet.push({ newG + h, newG, jpIdx });
            }
        }
    }

    // ========== 回溯路径 ==========
    vector<Hex> path;
    int idx = index(end);
    if (nodeData[idx].parentIdx < 0) return {};

    while (idx >= 0) {
        Hex h(idx % map.width, idx / map.width);
        path.push_back(h);
        idx = nodeData[idx].parentIdx;
    }
    reverse(path.begin(), path.end());
    return path;
}
