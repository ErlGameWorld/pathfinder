@echo off
echo Stopping previous server...
:: Kill C++ backend process just in case
taskkill /F /IM pathfinder_viz.exe 2>nul

cd web
node server.js