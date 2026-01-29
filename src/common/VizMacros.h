#pragma once

#include <iostream>
#include <vector>
#include "Hex.h"

// Global Visualization Macros
// controlled by VIZ_SUPPORT definition

#ifdef VIZ_SUPPORT
    extern std::vector<Hex> g_vizVisitedLog;
    extern bool g_vizEnabled;

    #define VIZ_START() g_vizEnabled = true; g_vizVisitedLog.clear();
    #define VIZ_STOP() g_vizEnabled = false;
    #define VIZ_LOG(hex) if (g_vizEnabled) g_vizVisitedLog.push_back(hex);
    
    #define VIZ_PRINT_RESULTS() \
        std::cout << "VISITED " << g_vizVisitedLog.size() << " "; \
        for (const auto& h : g_vizVisitedLog) { \
            Hex off = axialToOffset(h.q, h.r); \
            std::cout << off.q << "," << off.r << " "; \
        } \
        std::cout << std::endl;

#else
    #define VIZ_START()
    #define VIZ_STOP()
    #define VIZ_LOG(hex)
    #define VIZ_PRINT_RESULTS()
#endif
