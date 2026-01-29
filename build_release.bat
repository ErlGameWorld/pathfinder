@echo off
pushd "%~dp0"
if not exist ".\build" mkdir ".\build"

echo Building Release Version (No Visualization, Optimized)...
cl /EHsc /std:c++17 /O2 /DNDEBUG /utf-8 src\main.cpp src\map\HexMap.cpp src\map\MapGenerator.cpp src\finder\AStar.cpp src\finder\DHPAStar.cpp src\finder\BFS.cpp src\finder\BiAStar.cpp src\finder\JPS.cpp src\finder\FlowField.cpp src\finder\DHPAJps.cpp src\finder\DStarLite.cpp src\finder\HJPS.cpp /Fo".\build\\" /Fd".\build\vc140.pdb" /Fe".\build\pathfinder_viz.exe"

if %ERRORLEVEL% EQU 0 (
    echo Build Successful! Output: .\build\pathfinder_viz.exe
) else (
    echo Build Failed!
)
popd