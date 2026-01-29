#include "PathSmoother.h"
#include <algorithm>
#include <cmath>

using namespace std;

PathSmoother::PathSmoother(const HexMap& m) : map(m) {}

// -----------------------------------------------------
// 六边形 Bresenham 风格射线检测（严格无穿障碍）
// -----------------------------------------------------
bool PathSmoother::lineOfSight(const Hex& a, const Hex& b) const {
    int dq = b.q - a.q;
    int dr = b.r - a.r;
    int ds = -dq - dr;

    int N = max({abs(dq), abs(dr), abs(ds)});
    for (int i = 1; i < N; i++) {
        double t = double(i) / N;
        double q = a.q + dq * t;
        double r = a.r + dr * t;
        double s = -q - r;

        int rq = int(round(q));
        int rr = int(round(r));
        int rs = int(round(s));

        double q_diff = fabs(rq - q);
        double r_diff = fabs(rr - r);
        double s_diff = fabs(rs - s);

        if (q_diff > r_diff && q_diff > s_diff)
            rq = -rr - rs;
        else if (r_diff > s_diff)
            rr = -rq - rs;
        else
            rs = -rq - rr;

        Hex h(rq, rr);
        if (!map.inBounds(h) || !map.isWalkable(h))
            return false;
    }
    return true;
}

// -----------------------------------------------------
// Floyd Path Smoothing (工程级版本)
// -----------------------------------------------------
std::vector<Hex> PathSmoother::smooth(const std::vector<Hex>& path) {
    if (path.size() <= 2) return path;

    vector<Hex> result = path;
    int i = 0;
    while (i < (int)result.size() - 2) {
        int j = i + 2;
        while (j < (int)result.size()) {
            if (lineOfSight(result[i], result[j])) {
                result.erase(result.begin() + i + 1, result.begin() + j);
                j = i + 2;
            } else break;
        }
        i++;
    }
    return result;
}
