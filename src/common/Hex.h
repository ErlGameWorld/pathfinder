#pragma once

#include <cmath>
#include <functional>
#include <vector>
#include <cstdlib>

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
    int distance(const Hex& b) const {
        return (std::abs(q - b.q) + std::abs(q + r - b.q - b.r) + std::abs(r - b.r)) / 2;
    }

    Hex neighbor(int dir) const;
};

namespace std {
    template <> struct hash<Hex> {
        size_t operator()(const Hex& h) const {
            return (size_t(h.q) * 0x1f1f1f1f) ^ size_t(h.r);
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
