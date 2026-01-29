#include "HexMap.h"
#include <algorithm> // for std::fill

HexMap::HexMap(int w, int h) {
    // 复用 resize 逻辑进行初始化
    resize(w, h);
}

// 注意：构造函数中已经在 Header 里定义了 clustersX/Y 的计算逻辑吗？
// 没有，Header 里只是定义了变量，这里实现具体的计算。
void HexMap::resize(int w, int h) {
    width = w;
    height = h;
    
    // 重新分配 Grid
    grid.clear();
    grid.resize(w * h, 0); // 0 = Walkable

    // 计算分簇信息 (用于 HPA* 等分层算法)
    if (CLUSTER_SIZE > 0) {
        clustersX = (width + CLUSTER_SIZE - 1) / CLUSTER_SIZE;
        clustersY = (height + CLUSTER_SIZE - 1) / CLUSTER_SIZE;
    } else {
        clustersX = 1;
        clustersY = 1;
    }
}

// 输入的是标准的 Hex (Axial) 坐标
void HexMap::setObstacle(const Hex& h, bool blocked) {
    int idx = getIndex(h); // 自动处理 Axial -> Offset -> Index 转换
    if (idx != -1) {
        grid[idx] = blocked ? 1 : 0;
    }
}

// 专门给 UI 使用的接口，因为 UI 传来的通常是点击的行列号 (Offset)
void HexMap::setObstacleOffset(int col, int row, bool blocked) {
    int idx = getIndex(col, row);
    if (idx != -1) {
        grid[idx] = blocked ? 1 : 0;
    }
}

int HexMap::getClusterId(const Hex& h) const {
    // 必须先转为 Offset 坐标，因为 Cluster 是基于矩形网格划分的
    int col = h.q + (h.r - (h.r & 1)) / 2;
    int row = h.r;
    
    // 边界检查
    if (col < 0 || col >= width || row < 0 || row >= height) return -1;

    int cx = col / CLUSTER_SIZE;
    int cy = row / CLUSTER_SIZE;
    return cy * clustersX + cx;
}