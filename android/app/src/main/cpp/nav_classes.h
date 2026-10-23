#ifndef HSV2_NAV_CLASSES_H
#define HSV2_NAV_CLASSES_H

#include <cstdint>

#include "safety.h"

// Navigation classes merged from the 150 ADE20K classes (see semseg.cpp).
// Header-only so the PC replay tool shares it with the app.
enum NavClass : uint8_t {
    kNavObstacle = 0,  // furniture, objects, anything not listed below
    kNavFloor,         // floor, rug: walkable indoors
    kNavWall,          // wall, window, curtain, mirror, column, fence
    kNavDoor,
    kNavStairs,        // stairs, stairway, step, escalator
    kNavCeiling,
    kNavPerson,
    kNavVehicle,       // car, bus, truck, bicycle, ...
    kNavRoad,
    kNavSidewalk,      // sidewalk, path
    kNavTerrain,       // grass, earth, sand, field
    kNavWater,
    kNavSky,
    kNavVegetation,    // tree, plant, flower
    kNavBuilding,
    kNavCount
};

// Navigation class -> role used by the safety layer.
inline uint8_t navToSemRole(uint8_t c) {
    switch (c) {
        case kNavFloor: case kNavRoad: case kNavSidewalk: case kNavTerrain:
            return kSemWalkable;
        case kNavStairs:
            return kSemStairs;
        case kNavObstacle: case kNavVehicle:
            return kSemObstacle;
        default:
            return kSemOther;
    }
}

#endif  // HSV2_NAV_CLASSES_H
