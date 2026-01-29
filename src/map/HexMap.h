#pragma once

#include <vector>
#include <cstdint>
#include <cmath> // for abs if needed elsewhere
#include "../common/Hex.h"
#include "../common/Types.h"

class HexMap {
public:
    int width, height;
    int clustersX, clustersY;
    
    // 假设 Cluster 大小，如果没有全局定义，这里定义一个默认值
    static const int CLUSTER_SIZE = 20; 
    
    // 0: Walkable, 1: Obstacle
    // 使用一维数组存储二维数据 (Row-major order based on Offset coords)
    std::vector<uint8_t> grid; 

    HexMap(int w, int h);
    ~HexMap() = default;

    // ---------------------------------------------------------
    // 核心高性能内联函数 (供 BFS/A*/JPS 及其它算法使用)
    // ---------------------------------------------------------

    // 1. 将 Hex (Axial) 转换为一维数组索引 (Odd-R Offset)
    // 这是所有寻路算法的第一步，必须极快且统一
    inline int getIndex(const Hex& h) const {
        // Odd-R 转换公式: col = q + (r - (r&1)) / 2
        // 注意：这里修复了原代码直接 return r*w+q 的逻辑错误
        int col = h.q + (h.r - (h.r & 1)) / 2;
        int row = h.r;

        // 边界检查
        if (col < 0 || col >= width || row < 0 || row >= height) {
            return -1;
        }
        return row * width + col;
    }

    // 2. 将 Offset (Col, Row) 转换为一维数组索引
    // 用于 UI 交互或直接基于行列的逻辑
    inline int getIndex(int col, int row) const {
        if (col < 0 || col >= width || row < 0 || row >= height) {
            return -1;
        }
        return row * width + col;
    }

    // 3. 将一维数组索引还原为 Hex (Axial)
    // 用于寻路结束后重建路径
    inline Hex getHex(int index) const {
        int r = index / width;
        int c = index % width;
        int q = c - (r - (r & 1)) / 2;
        return Hex(q, r);
    }

    // 4. 极速检查：直接查表 (配合 BFS 优化)
    // 调用者需确保 index 有效 (不为 -1)
    inline bool isWalkableIdx(int index) const {
        return grid[index] == 0;
    }

    // 5. 通用检查：带坐标转换和边界检查
    inline bool isWalkable(const Hex& h) const {
        int idx = getIndex(h);
        return idx != -1 && grid[idx] == 0;
    }

    // 6. 边界检查
    inline bool inBounds(const Hex& h) const {
        return getIndex(h) != -1;
    }

    // ---------------------------------------------------------
    // 普通成员函数 (实现放在 .cpp)
    // ---------------------------------------------------------

    // 设置障碍物 (输入为 Axial Hex)
    void setObstacle(const Hex& h, bool blocked);

    // 设置障碍物 (输入为 Offset Col, Row) - 方便 UI 调用
    void setObstacleOffset(int col, int row, bool blocked);

    int getClusterId(const Hex& h) const;

    void resize(int w, int h);
};