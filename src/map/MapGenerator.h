#pragma once

#include <vector>
#include <random>
#include <string>
#include "HexMap.h"
#include "../common/Types.h"

struct MapGenerator {
    int width, height;
    std::vector<std::vector<TerrainType>> terrain;
    std::vector<std::vector<int>> elevation;
    std::mt19937 rng;
    
    MapGenerator(int w, int h);
    
    double noise2D(int x, int y);
    double smoothNoise(int x, int y);
    double interpolatedNoise(double x, double y);
    double noise(int x, int y, int scale, double persistence, int octaves);
    
    void generateAll();
    void applyToMap(HexMap& hexMap);
    std::string serializeRLE();
};
