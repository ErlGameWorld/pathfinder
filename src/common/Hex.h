//Hex.h
#pragma once

#include <cmath>
#include <functional>
#include <vector>
#include <cstdlib>
#include <algorithm>

struct Hex {
    int q, r;
    Hex(int q_ = 0, int r_ = 0) : q(q_), r(r_) {}
    
    bool operator==(const Hex& other) const { return q == other.q && r == other.r; }
    bool operator!=(const Hex& other) const { return !(*this == other); }
    bool operator<(const Hex& other) const { 
        if (q != other.q) return q < other.q;
        return r < other.r;
    }
    
    Hex operator+(const Hex& other) const { return Hex(q + other.q, r + other.r); }
    Hex operator-(const Hex& other) const { return Hex(q - other.q, r - other.r); }
    
    // Manhattan distance (Hex grid)
    //int distance(const Hex& b) const {
    //    return (std::abs(q - b.q) + std::abs(q + r - b.q - b.r) + std::abs(r - b.r)) / 2;
    //}

    // 六边形曼哈顿距离
    int distance(const Hex& o) const {
        int dq = o.q - q, dr = o.r - r, ds = -dq - dr;
        return std::max({std::abs(dq), std::abs(dr), std::abs(ds)});
    }

    Hex neighbor(int dir) const;
};

namespace std {
    template <> struct hash<Hex> {
        size_t operator()(const Hex& h) const noexcept {
            // 使用更好的哈希组合方式，减少冲突
            // 基于 Thomas Wang 的整数哈希算法
            size_t hq = static_cast<size_t>(h.q);
            size_t hr = static_cast<size_t>(h.r);
            hq = (hq ^ 61) ^ (hq >> 16);
            hq = hq + (hq << 3);
            hq = hq ^ (hq >> 4);
            hq = hq * 0x27d4eb2d;
            hq = hq ^ (hq >> 15);
            return hq ^ (hr + 0x9e3779b9 + (hq << 6) + (hq >> 2));
        }
    };
}

const Hex HEX_DIRS[6] = {
    Hex(1, 0), Hex(1, -1), Hex(0, -1), Hex(-1, 0), Hex(-1, 1), Hex(0, 1)
};

inline Hex Hex::neighbor(int dir) const {
    return *this + HEX_DIRS[dir];
}

// Helper to convert Offset (Odd-R) to Axial
inline Hex offsetToAxial(int col, int row) {
    int q = col - (row - (row & 1)) / 2;
    int r = row;
    return Hex(q, r);
}

// Helper to convert Axial to Offset (Odd-R)
inline Hex axialToOffset(int q, int r) {
    int col = q + (r - (r & 1)) / 2;
    int row = r;
    return Hex(col, row);
}
