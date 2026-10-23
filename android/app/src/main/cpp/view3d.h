#ifndef HSV2_VIEW3D_H
#define HSV2_VIEW3D_H

#include <cstdint>

#include "local_map.h"
#include "nav_plan.h"

/*
 * Phase 4b 3D view: the dynamic map seen from a virtual camera behind and
 * above the user (chase view), rendered on the CPU into an RGBA8 image.
 *
 *  - floor: grey shaded by walking cost (light = open, dark = near walls),
 *    orange drop-offs, yellow stairs, dark = unknown;
 *  - obstacles: red columns with their measured height;
 *  - live point cloud (latest frame) coloured by height;
 *  - trajectory: red arrows; current pose: RGB axes;
 *  - planned path: thick green line.
 */
struct View3DOptions {
    float back{2.0f};      // virtual camera distance behind the user (m)
    float up{4.5f};        // height above the floor (m)
    float lookAhead{2.5f}; // look-at point ahead of the user (m)
    float behindCull{0.3f};// obstacles / points more than this behind the user are not drawn
    float fovScale{0.75f}; // focal length as a fraction of the image width
    double tSec{-1};       // frame time, for time-based camera smoothing
    // Path display (from guidance): 1 = verified safe (green), 2 = planned but
    // not verified (dashed amber, no goal), 0 = hidden.
    int pathState{1};
    float stopAheadDist{-1}; // > 0: obstacle directly ahead at this distance (red bar + X)
};

// Smoothed chase camera (kept between frames so head nods / turns don't jerk
// the view). Pass the same instance every frame; nullptr = no smoothing.
struct View3DState {
    bool init{false};
    double lastT{-1};
    float ux{0}, uy{0}, hx{0}, hy{1};
};

void renderView3D(uint8_t* rgba, int width, int height, int stride, const LocalMap& map, const CostMap* cost,
                  const PlanResult* plan, const View3DOptions& opt = View3DOptions(),
                  View3DState* state = nullptr);

#endif  // HSV2_VIEW3D_H
