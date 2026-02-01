# Hex Grid Pathfinder
Odd-R 布局（奇数行偏移）
这是一个高性能的六边形网格（Hex Grid）寻路算法实验平台。集成了多种经典与现代寻路算法，支持复杂地形生成、Web 可视化、性能基准测试以及动态环境交互。

## 🚀 核心特性

*   **多算法支持**：从基础 BFS 到高级分层寻路，涵盖多种场景。
*   **超大地图支持**：支持 2500x2500 甚至更大的六边形网格地图。
*   **Web 可视化**：
    *   实时动画展示寻路过程（访问节点、搜索边界）。
    *   交互式地图编辑（绘制墙壁、擦除、设置起终点）。
    *   平滑的缩放与平移操作。
*   **地图生成系统**：基于噪声（Perlin Noise）生成山脉、河流、森林、城市等复杂地形。
*   **性能基准测试**：一键对比所有已实现算法在当前地图上的运行效率。
*   **C++ 高性能后端**：核心算法采用 C++17 编写，确保极致性能。

## 🧠 支持的算法

本项目实现了以下寻路算法，可在界面中实时切换。所有算法代码均位于 `src/finder/` 目录下：

1.  **BFS (Breadth-First Search)**
    *   **描述**: 广度优先搜索，用于寻找无权图的最短路径，常作为基准对比。
    *   **实现**: [BFS.cpp](src/finder/BFS.cpp)。使用 `std::vector` 手动模拟队列以优化内存分配，性能优于标准 `std::queue`。

2.  **A\*** (A-Star)
    *   **描述**: 经典的启发式搜索算法，寻找最短路径的标准方案。
    *   **实现**: [AStar.cpp](src/finder/AStar.cpp)。采用二叉堆 (`std::make_heap`) 优化的优先队列，使用曼哈顿距离作为启发函数。

3.  **Bi-Directional A\***
    *   **描述**: 双向 A*，同时从起点和终点进行搜索，通常比单向 A* 更快相遇。
    *   **实现**: [BiAStar.cpp](src/finder/BiAStar.cpp)。使用位掩码 (`visitMask`) 标记正向/反向访问状态，并复用内存池。

4.  **JPS (Jump Point Search)**
    *   **描述**: 针对六边形网格优化的跳点搜索算法，在开阔区域能极大减少搜索节点数。
    *   **实现**: [JPS.cpp](src/finder/JPS.cpp)。基础版本的 Hex JPS 实现，主要进行直线扫描。

5.  **HJPS (Hierarchical/Hex JPS)**
    *   **描述**: 基于论文《基于正六边形栅格 JPS 算法的智能路径规划》的改进版 JPS。
    *   **实现**: [HJPS.cpp](src/finder/HJPS.cpp)。实现了递归侧向探测 (Recursive Lateral Check) 和严格的强制邻居剪枝规则，在复杂地形下比普通 JPS 更健壮。

6.  **Flow Field**
    *   **描述**: 流场算法 (Vector Field)，基于 Dijkstra 生成全局矢量场。
    *   **实现**: [FlowField.cpp](src/finder/FlowField.cpp)。适合 RTS 游戏中海量单位共享同一个目标点，一次计算即可服务所有单位。

7.  **DHPA\*** (Dynamic Hierarchical Pathfinding A*)
    *   **描述**: 分层寻路算法。将地图抽象为聚类图（Cluster Graph），在大地图长距离寻路中性能卓越。
    *   **实现**: [DHPAStar.cpp](src/finder/DHPAStar.cpp)。将地图分割为 Block，构建抽象图进行搜索。

8.  **DHPA-JPS**
    *   **描述**: DHPA* 的变体。
    *   **实现**: [DHPAJps.cpp](src/finder/DHPAJps.cpp)。在分层图的底层构建和局部路径细化中使用 JPS 加速。

## � 算法选择建议

根据不同的应用场景，推荐使用以下算法：

*   **标准方案**：**A\*** (配合立方体坐标系 + 二叉堆优化)。这是最通用且稳定的选择，能满足 90% 的需求。
*   **单位极多 (RTS/群组寻路)**：**Flow Fields** (流场算法)。一次计算即可支持成百上千个单位向同一目标移动。
*   **超大地图 (长距离)**：**DHPA\*** (分层寻路)。通过预处理抽象图，极大减少长距离搜索的节点数。
*   **极致单体速度 (开阔地)**：**Hexagonal JPS**。如果地图开阔区域较多，JPS 能比 A* 快数倍。

## 📂 目录结构

```text
e:/pathfinder/
├── src/
│   ├── finder/         # 寻路算法实现 (AStar, JPS, DHPA* 等)
│   ├── common/         # 通用数据结构 (Hex, Types, Macros)
│   ├── map/            # 地图逻辑 (HexMap, MapGenerator)
│   └── main.cpp        # 程序入口与协议处理
├── web/                # Web 前端与 Node.js 服务器
│   ├── index.html      # 可视化界面
│   └── server.js       # 中间件服务器 (连接 Browser 与 C++ EXE)
├── build_debug.bat     # 编译调试版 (开启可视化日志)
├── build_release.bat   # 编译发布版 (极致性能，无中间日志)
└── run_server.bat      # 启动服务器脚本
```

## 🛠️ 如何编译与运行

### 1. 环境要求
*   **编译器**: 支持 C++17 的 MSVC (Visual Studio 2019/2022) 或其他编译器。
*   **运行时**: Node.js (用于运行 Web 服务器)。

### 2. 编译后端
在项目根目录下运行以下脚本（会自动检测 MSVC 环境）：

*   **编译发布版 (推荐)**:
    ```powershell
    .\build_release.bat
    ```
    *生成的 `pathfinder_viz.exe` 位于 `build/Release/`。此版本经过 O2 优化，适合基准测试。*

*   **编译调试版**:
    ```powershell
    .\build_debug.bat
    ```
    *生成的 `pathfinder_viz.exe` 位于 `build/Debug/`。此版本包含可视化所需的详细日志输出。*

### 3. 启动
直接运行根目录下的启动脚本：
```powershell
.\run_server.bat
```
该脚本会自动调用 Node.js 启动 `web/server.js`，并拉起编译好的 C++ 后端。

### 4. 使用
打开浏览器访问：[http://localhost:8080](http://localhost:8080)

*   **Generate Map**: 输入自定义尺寸（如 800x600）或选择预设尺寸生成地图。
*   **Controls**:
    *   **左键**: 绘制墙壁 / 设置起点 / 设置终点（取决于当前模式）。
    *   **右键**: 擦除。
    *   **滚轮**: 缩放地图。
    *   **Shift + 拖动**: 平移地图。
*   **Algorithm**: 在下拉菜单中选择算法，点击 `▶ 开始寻路`。
*   **Benchmark**: 点击 `📊 性能对比` 查看所有算法在当前起终点下的耗时和路径长度。

## 📝 交互协议 (Stdio)

C++ 后端通过标准输入输出 (Stdio) 与 Node.js 服务器通信。支持以下指令：

*   `G [w h]`: 生成地图 (Generate)。
*   `P c1 r1 c2 r2 [algo]`: 寻路 (Pathfind)。
*   `O c r state`: 更新障碍物 (Obstacle)。
*   `B c1 r1 c2 r2`: 运行基准测试 (Benchmark)。
*   `L [data]`: 加载地图数据 (Load)。

## 🗺️ 地图文件格式

本平台支持导入/导出 JSON 格式的地图文件。你可以根据以下规范生成地图文件，以便在其他工具或游戏中使用。

### JSON 结构

文件应为包含 `width`, `height`, `walls` 字段的 JSON 对象。

```json
{
  "width": 250,          // 地图宽度 (列数)
  "height": 250,         // 地图高度 (行数)
  "walls": [             // 地形数据 (扁平数组)
    413, 3, 3,           // 格式: [col, row, type, col, row, type, ...]
    149, 6, 2,
    ...
  ]
}
```

*   **width / height**: 地图的网格尺寸。
*   **walls**: 一个一维整数数组。**仅记录非空地块（稀疏存储）**。每 3 个数字代表一个地块：
    *   第 1 个数: **列索引 (q / col)**
    *   第 2 个数: **行索引 (r / row)**
    *   第 3 个数: **地块类型 ID (Type ID)**

### 地块类型 (Terrain Types)

| ID | 类型 | 含义 | 备注 |
| :--- | :--- | :--- | :--- |
| **0** | Empty / Plain | 空地 / 平原 | **默认地形，不会出现在 `walls` 数组中** |
| **1** | Mountain | 山脉 | 阻挡 |
| **2** | River | 河流 | 阻挡 (部分逻辑可能视为高代价) |
| **3** | City | 城市 | 阻挡 (或高权重节点) |
| **4** | Pass | 关隘 | 可通行 (通常位于山脉之间) |
| **5** | Forest | 森林 | 阻挡 (或高代价) |
| **100** | User Wall | 墙壁 | 用户手动绘制的阻挡 |
| **其他** | Obstacle | 障碍物 | 其他未定义 ID 默认视为通用障碍物 |

---
*Created by Trae AI Assistant*
