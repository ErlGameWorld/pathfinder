#include "MapGenerator.h"
#include <iostream>
#include <sstream>

MapGenerator::MapGenerator(int w, int h) : width(w), height(h), rng(std::random_device{}()) {
    terrain.resize(h, std::vector<TerrainType>(w, PLAIN));
    elevation.resize(h, std::vector<int>(w, 0));
}

double MapGenerator::noise2D(int x, int y) {
    int n = x + y * 57 + 131 * (x * y);
    n = (n << 13) ^ n;
    int t = (n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff;
    return (double)t / 2147483647.0; 
}

double MapGenerator::smoothNoise(int x, int y) {
    double corners = (noise2D(x-1, y-1) + noise2D(x+1, y-1) + noise2D(x-1, y+1) + noise2D(x+1, y+1)) / 16.0;
    double sides = (noise2D(x-1, y) + noise2D(x+1, y) + noise2D(x, y-1) + noise2D(x, y+1)) / 8.0;
    double center = noise2D(x, y) / 4.0;
    return corners + sides + center;
}

double MapGenerator::interpolatedNoise(double x, double y) {
    int intX = int(x); int intY = int(y);
    double fracX = x - intX; double fracY = y - intY;
    
    double v1 = smoothNoise(intX, intY); 
    double v2 = smoothNoise(intX + 1, intY);
    double v3 = smoothNoise(intX, intY + 1); 
    double v4 = smoothNoise(intX + 1, intY + 1);
    
    double i1 = v1 * (1 - fracX) + v2 * fracX; 
    double i2 = v3 * (1 - fracX) + v4 * fracX;
    return i1 * (1 - fracY) + i2 * fracY;
}

double MapGenerator::noise(int x, int y, int scale, double persistence, int octaves) {
    double total = 0; double freq = 1.0 / scale; double amp = 1.0; double maxV = 0;
    for (int i = 0; i < octaves; i++) {
        total += interpolatedNoise(x * freq, y * freq) * amp;
        maxV += amp; 
        amp *= persistence; 
        freq *= 2.0;
    }
    return total / maxV;
}

void MapGenerator::generateAll() {
    std::cerr << "[MapGen] Generating terrain features (complex)..." << std::endl;
    
    // 1. Terrain Base (Layered Noise)
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // Base elevation
            double elev = noise(x, y, 150, 0.5, 6); // More octaves for detail
            // Moisture/Biome
            double moisture = noise(x + 5000, y + 5000, 200, 0.5, 4);
            // Roughness/Details
            double rough = noise(x + 9000, y + 9000, 30, 0.6, 2);

            elevation[y][x] = int(elev * 100);
            
            // Adjusted Thresholds for more obstacles
            if (elev > 0.60) {
                 // High mountains
                 terrain[y][x] = MOUNTAIN;
                 // Occasional passes in mountains
                 if (rough > 0.7 && elev < 0.7) terrain[y][x] = PASS; 
            }
            else if (elev > 0.35 && moisture < 0.4) {
                terrain[y][x] = FOREST;
            }
            else if (moisture > 0.75) {
                // Lakes / Swamps
                terrain[y][x] = RIVER; 
            }
            else {
                terrain[y][x] = PLAIN;
                // Random small obstacles (rocks/trees) in plains
                if (rough > 0.65) terrain[y][x] = FOREST;
            }
        }
    }
    
    // 2. Rivers (More active)
    std::cerr << "[MapGen] Generating rivers..." << std::endl;
    std::uniform_int_distribution<int> distX(0, width - 1), distY(0, height - 1);
    int riverCount = 50; 
    for (int i = 0; i < riverCount; ++i) {
        Hex cur(distX(rng), distY(rng));
        int safety = 0;
        // Rivers start at high elevation (but not peak) and flow down
        if (elevation[cur.r][cur.q] < 30) continue; 

        while (cur.q >= 0 && cur.q < width && cur.r >= 0 && cur.r < height && safety++ < 1000) {
            if (terrain[cur.r][cur.q] != MOUNTAIN) terrain[cur.r][cur.q] = RIVER;
            
            int bestD = -1, bestE = elevation[cur.r][cur.q];
            
            // Flow to lowest neighbor
            for (int d = 0; d < 6; ++d) {
                Hex n = cur + HEX_DIRS[d];
                if (n.q>=0 && n.q<width && n.r>=0 && n.r<height) {
                     if (elevation[n.r][n.q] < bestE) {
                        bestE = elevation[n.r][n.q]; 
                        bestD = d;
                    }
                }
            }
            
            if (bestD == -1) break; 
            cur = cur + HEX_DIRS[bestD];
        }
    }
    
    // 3. Cities
    std::cerr << "[MapGen] Placing cities..." << std::endl;
    int cityCnt = 0;
    int attempts = 0;
    while(cityCnt < 150 && attempts++ < 200000) { 
        int x = distX(rng), y = distY(rng);
        if (terrain[y][x] == PLAIN || terrain[y][x] == FOREST) { 
            terrain[y][x] = CITY; 
            cityCnt++; 
        }
    }
    std::cerr << "[MapGen] Placed " << cityCnt << " cities." << std::endl;
}

void MapGenerator::applyToMap(HexMap& hexMap) {
    int obs = 0;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // Blocks: MOUNTAIN, RIVER, FOREST
            // Allow: PLAIN, CITY, PASS
            if (terrain[y][x] == MOUNTAIN || terrain[y][x] == RIVER || terrain[y][x] == FOREST) {
                hexMap.setObstacleOffset(x, y, true);
                obs++;
            }
        }
    }
    std::cerr << "[MapGen] Applied. Obstacles: " << obs << " (" << (obs*100.0/(width*height)) << "%)" << std::endl;
}

std::string MapGenerator::serializeRLE() {
    std::stringstream ss;
    ss << width << " " << height << " ";
    
    int currentType = -1;
    int count = 0;
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int type = terrain[y][x];
            
            if (type == currentType) {
                count++;
            } else {
                if (currentType != -1) {
                    ss << count << " " << currentType << " ";
                }
                currentType = type;
                count = 1;
            }
        }
    }
    if (count > 0) {
        ss << count << " " << currentType << " ";
    }
    return ss.str();
}
