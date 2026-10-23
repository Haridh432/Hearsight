// Host test: Phase 4b cost map + human-centred planner on maps built by the
// synthetic renderer of test_map.cpp.
#define main test_map_main
#include "test_map.cpp"
#undef main
#include "nav_plan.h"
#include "view3d.h"
#include <cstdlib>

static float pathMaxAbsX(const PlanResult& r, const Sim& s, float zMax) {
    // Waypoints are in map-plane coordinates; convert true world (x, z) to
    // plane with toPlane for comparison: sample along the polyline.
    float worst = 0;
    for (size_t i = 2; i + 1 < r.path.size(); i += 2) {
        for (int t = 0; t <= 10; ++t) {
            const float a = r.path[i - 2] + (r.path[i] - r.path[i - 2]) * t / 10;
            const float b = r.path[i - 1] + (r.path[i + 1] - r.path[i - 1]) * t / 10;
            // Find the world x, z for plane (a, b): the plane basis is linear,
            // so invert with two probes.
            float ax, bx, az, bz, a0, b0;
            const float o[3] = {0, 0, 0}, ex[3] = {1, 0, 0}, ez[3] = {0, 0, 1};
            s.map.toPlane(o, a0, b0);
            s.map.toPlane(ex, ax, bx); ax -= a0; bx -= b0;
            s.map.toPlane(ez, az, bz); az -= a0; bz -= b0;
            const float det = ax * bz - az * bx;
            const float x = ((a - a0) * bz - az * (b - b0)) / det;
            const float z = (ax * (b - b0) - (a - a0) * bx) / det;
            if (z < zMax) worst = std::max(worst, std::fabs(x));
        }
    }
    return worst;
}

// Which side of x = 0 the path passes at world depth z (sign of x there), 0 if unknown.
static int pathSideX(const PlanResult& r, const Sim& s, float z0) {
    const float o[3] = {0, 0, 0}, ex[3] = {1, 0, 0}, ez[3] = {0, 0, 1};
    float a0, b0, ax, bx, az, bz;
    s.map.toPlane(o, a0, b0);
    s.map.toPlane(ex, ax, bx); ax -= a0; bx -= b0;
    s.map.toPlane(ez, az, bz); az -= a0; bz -= b0;
    const float det = ax * bz - az * bx;
    float best = 1e9f, sx = 0;
    for (size_t i = 2; i + 1 < r.path.size(); i += 2)
        for (int t = 0; t <= 10; ++t) {
            const float a = r.path[i - 2] + (r.path[i] - r.path[i - 2]) * t / 10;
            const float b = r.path[i - 1] + (r.path[i + 1] - r.path[i - 1]) * t / 10;
            const float x = ((a - a0) * bz - az * (b - b0)) / det;
            const float z = (ax * (b - b0) - (a - a0) * bx) / det;
            if (std::fabs(z - z0) < best) { best = std::fabs(z - z0); sx = x; }
        }
    return best > 0.5f ? 0 : (sx > 0 ? 1 : -1);
}

static void worldDir(const Sim& s, float wx, float wz, float& dx, float& dy) {
    const float v[3] = {wx, 0, wz};
    s.map.toMapDir(v, dx, dy);
    const float l = std::hypot(dx, dy);
    dx /= l; dy /= l;
}

int main() {
    int fail = 0;
    char buf[256];
    auto check = [&](const char* name, bool ok, const char* d) {
        printf("%-50s: %s  %s\n", name, d, ok ? "OK" : "FAIL");
        if (!ok) ++fail;
    };
    // L-shaped corridor: 2 m wide along +z, turning right (+x) at z 3..5.
    Scene L;
    L.boxes.push_back({-1.1f, 0, -1, -1.0f, 2.2f, 5.1f, 1});   // left wall
    L.boxes.push_back({1.0f, 0, -1, 1.1f, 2.2f, 3.0f, 1});     // right wall up to the corner
    L.boxes.push_back({-1.1f, 0, 5.0f, 7.0f, 2.2f, 5.1f, 1});  // far wall (and the 2nd corridor's left wall)
    L.boxes.push_back({1.0f, 0, 2.9f, 7.0f, 2.2f, 3.0f, 1});   // 2nd corridor's right wall
    Sim s;
    for (int f = 0; f < 60; ++f) s.frame(makeCam(0, 1.6f, 1.5f * f / 60, 0, 27), L);          // walk in
    for (int f = 0; f < 45; ++f) s.frame(makeCam(0, 1.6f, 1.5f, 70.0f * f / 45, 27), L);     // look right
    for (int f = 0; f < 45; ++f) s.frame(makeCam(0, 1.6f, 1.5f, 70.0f - 70.0f * f / 45, 27), L);
    CostMap cm;
    Planner pl;
    cm.build(s.map);

    float dx, dy;
    worldDir(s, 0, 1, dx, dy);
    pl.reset();
    PlanResult a = pl.plan(s.map, cm, dx, dy);
    const float offCentre = pathMaxAbsX(a, s, 2.8f);
    snprintf(buf, sizeof buf, "ok %d, %zu waypoints, %.2f m, clearance %.2f m, |x| <= %.2f m, %.2f ms", a.ok,
             a.path.size() / 2, a.length, a.minClearance, offCentre, a.ms);
    check("1 straight ahead: stays in the corridor centre", a.ok && a.minClearance >= 0.5f && offCentre < 0.25f, buf);

    worldDir(s, 1, 1, dx, dy);
    pl.reset();
    PlanResult b = pl.plan(s.map, cm, dx, dy);
    if (getenv("OUT3D")) {
        std::vector<uint8_t> img(640 * 480 * 4, 0);
        renderView3D(img.data(), 640, 480, 640, s.map, &cm, &b);
        FILE* f = fopen(getenv("OUT3D"), "wb");
        fprintf(f, "P6\n640 480\n255\n");
        for (int i = 0; i < 640 * 480; ++i) fwrite(&img[i * 4], 1, 3, f);
        fclose(f);
    }
    // Goal in world coordinates.
    float gx, gz;
    {
        float a0, b0, ax, bx, az, bz;
        const float o[3] = {0, 0, 0}, ex[3] = {1, 0, 0}, ez[3] = {0, 0, 1};
        s.map.toPlane(o, a0, b0); s.map.toPlane(ex, ax, bx); ax -= a0; bx -= b0;
        s.map.toPlane(ez, az, bz); az -= a0; bz -= b0;
        const float det = ax * bz - az * bx;
        gx = ((b.goalX - a0) * bz - az * (b.goalY - b0)) / det;
        gz = (ax * (b.goalY - b0) - (b.goalX - a0) * bx) / det;
    }
    snprintf(buf, sizeof buf, "ok %d, goal (x %.2f, z %.2f), clearance %.2f m, turn %.0f deg, %.2f ms", b.ok, gx, gz,
             b.minClearance, b.turnDeg, b.ms);
    check("2 turn into the side corridor around the corner", b.ok && gx > 1.5f && gz > 3.1f && gz < 4.9f &&
          b.minClearance >= 0.35f, buf);

    // Narrow doorway 0.75 m wide in a wall 2.5 m ahead: must be passable.
    {
        Scene Dw;
        Dw.boxes.push_back({-3.0f, 0, 2.5f, -0.375f, 2.2f, 2.6f, 1});
        Dw.boxes.push_back({0.375f, 0, 2.5f, 3.0f, 2.2f, 2.6f, 1});
        Sim sd;
        for (int f = 0; f < 45; ++f) sd.frame(makeCam(0, 1.6f, 0.6f * f / 45, 0, 27), Dw);
        for (int f = 0; f < 30; ++f) sd.frame(makeCam(0, 1.6f, 0.6f, 0, 15), Dw);
        CostMap cmd; cmd.build(sd.map);
        worldDir(sd, 0, 1, dx, dy);
        pl.reset();
        PlanResult pd = pl.plan(sd.map, cmd, dx, dy);
        float gxd, gzd;
        {
            float a0, b0, ax, bx, az, bz;
            const float o[3] = {0, 0, 0}, ex[3] = {1, 0, 0}, ez[3] = {0, 0, 1};
            sd.map.toPlane(o, a0, b0); sd.map.toPlane(ex, ax, bx); ax -= a0; bx -= b0;
            sd.map.toPlane(ez, az, bz); az -= a0; bz -= b0;
            const float det = ax * bz - az * bx;
            gxd = ((pd.goalX - a0) * bz - az * (pd.goalY - b0)) / det;
            gzd = (ax * (pd.goalY - b0) - (pd.goalX - a0) * bx) / det;
        }
        snprintf(buf, sizeof buf, "ok %d blocked %d, goal z %.2f, clearance %.2f m", pd.ok, pd.blocked, gzd, pd.minClearance);
        check("5 narrow doorway (0.75 m) is passable", pd.ok && gzd > 2.7f, buf);
        (void)gxd;
    }
    // Box in the corridor centre at z 2.6-3.0: path goes around it.
    Scene B;
    B.boxes.push_back({-1.1f, 0, -1, -1.0f, 2.2f, 8, 1});
    B.boxes.push_back({1.0f, 0, -1, 1.1f, 2.2f, 8, 1});
    B.boxes.push_back({-0.3f, 0, 2.6f, 0.3f, 0.8f, 3.0f, 1});
    Sim s2;
    for (int f = 0; f < 45; ++f) s2.frame(makeCam(0, 1.6f, 0.8f * f / 45, 0, 27), B);
    for (int f = 0; f < 30; ++f) s2.frame(makeCam(0, 1.6f, 0.8f, 0, 20), B);
    cm.build(s2.map);
    worldDir(s2, 0, 1, dx, dy);
    pl.reset();
    PlanResult c = pl.plan(s2.map, cm, dx, dy);
    if (getenv("DBG")) {
        float a0, b0, ax, bx, az, bz;
        const float o[3] = {0, 0, 0}, ex[3] = {1, 0, 0}, ez[3] = {0, 0, 1};
        s2.map.toPlane(o, a0, b0); s2.map.toPlane(ex, ax, bx); ax -= a0; bx -= b0;
        s2.map.toPlane(ez, az, bz); az -= a0; bz -= b0;
        const float det = ax * bz - az * bx;
        for (size_t i = 0; i + 1 < c.path.size(); i += 2) {
            const float x = ((c.path[i] - a0) * bz - az * (c.path[i + 1] - b0)) / det;
            const float z = (ax * (c.path[i + 1] - b0) - (c.path[i] - a0) * bx) / det;
            printf("   wp x %.2f z %.2f\n", x, z);
        }
    }
    snprintf(buf, sizeof buf, "ok %d, clearance %.2f m, first turn %.0f deg, %.2f ms", c.ok, c.minClearance, c.turnDeg, c.ms);
    check("3 box in the corridor: path goes around it", c.ok && c.minClearance >= 0.25f && std::fabs(c.turnDeg) > 5, buf);

    // Dead end: walls 1.2 m ahead and on both sides; the only free way is
    // back. The planner must say "blocked", not plan a U-turn detour.
    {
        Scene D;
        D.boxes.push_back({-1.1f, 0, -3, -1.0f, 2.2f, 2.3f, 1});
        D.boxes.push_back({1.0f, 0, -3, 1.1f, 2.2f, 2.3f, 1});
        D.boxes.push_back({-1.1f, 0, 2.2f, 1.1f, 2.2f, 2.3f, 1});
        Sim s3;
        for (int f = 0; f < 30; ++f) s3.frame(makeCam(0, 1.6f, -2.0f + 1.0f * f / 30, 180, 27), D);   // walk in backwards: see behind
        for (int f = 0; f < 45; ++f) s3.frame(makeCam(0, 1.6f, -1.0f, 180.0f - 180.0f * f / 45, 27), D);
        for (int f = 0; f < 30; ++f) s3.frame(makeCam(0, 1.6f, -1.0f + 1.5f * f / 30, 0, 27), D);
        cm.build(s3.map);
        worldDir(s3, 0, 1, dx, dy);
        pl.reset();
        PlanResult d = pl.plan(s3.map, cm, dx, dy);
        snprintf(buf, sizeof buf, "ok %d blocked %d (length %.1f m, turn %.0f deg)", d.ok, d.blocked, d.length, d.turnDeg);
        check("4 dead end ahead: 'blocked', no U-turn detour", !d.ok && (d.blocked || d.path.empty()), buf);
    }
    // 3D view before the first plan: empty cost map / no plan must not crash.
    {
        CostMap empty; PlanResult none;
        std::vector<uint8_t> img(640 * 480 * 4, 0);
        renderView3D(img.data(), 640, 480, 640, s.map, &empty, &none);
        renderView3D(img.data(), 640, 480, 640, s.map, nullptr, nullptr);
        check("6 3D view with an empty cost map does not crash", true, "rendered");
    }
    // 7. Stability: in the straight corridor, replan 30 times while the
    //    head swings +-30 deg (direction jitter). Goal and turn must hold.
    {
        pl.reset();
        cm.build(s.map);
        float gx0 = 0, gy0 = 0, maxJump = 0, wx0 = 0, wy0 = 0, maxWp = 0;
        std::mt19937 rj(4);
        std::uniform_real_distribution<float> J(-30, 30);
        for (int k = 0; k < 30; ++k) {
            const float a = J(rj) * 3.14159265f / 180;
            float wx = std::sin(a), wz = std::cos(a);
            worldDir(s, wx, wz, dx, dy);
            PlanResult p = pl.plan(s.map, cm, dx, dy);
            if (!p.ok) continue;
            // First waypoint >= 1 m from the user.
            float fx = p.path[p.path.size() - 2], fy = p.path.back();
            for (size_t q = 2; q + 1 < p.path.size(); q += 2)
                if (std::hypot(p.path[q] - p.path[0], p.path[q + 1] - p.path[1]) >= 1.0f) { fx = p.path[q]; fy = p.path[q + 1]; break; }
            if (k == 0) { gx0 = p.goalX; gy0 = p.goalY; wx0 = fx; wy0 = fy; }
            maxJump = std::max(maxJump, std::hypot(p.goalX - gx0, p.goalY - gy0));
            maxWp = std::max(maxWp, std::hypot(fx - wx0, fy - wy0));
        }
        snprintf(buf, sizeof buf, "goal moved at most %.2f m, path (first metre) at most %.2f m", maxJump, maxWp);
        check("7 head swinging +-30 deg: goal and path stay steady", maxJump < 0.3f && maxWp < 0.3f, buf);
    }
    // D* Lite incremental re-planning: a clear corridor, then a box appears on the
    // path (a person stepping in / a newly seen obstacle), then the user walks on.
    // Each replan reuses the search; its result must match a fresh full search.
    {
        Scene clear;
        clear.boxes.push_back({-1.1f, 0, -1, -1.0f, 2.2f, 9, 1});
        clear.boxes.push_back({1.0f, 0, -1, 1.1f, 2.2f, 9, 1});
        Scene withBox = clear;
        withBox.boxes.push_back({-0.35f, 0, 2.2f, 0.25f, 1.7f, 2.6f, 1});   // ~person-sized
        Sim si;
        for (int f = 0; f < 45; ++f) si.frame(makeCam(0, 1.6f, 0.5f * f / 45, 0, 27), clear);
        CostMap c1; c1.build(si.map);
        float ddx, ddy; worldDir(si, 0, 1, ddx, ddy);
        Planner inc; inc.reset();
        PlanResult p0 = inc.plan(si.map, c1, ddx, ddy);
        const float straight0 = pathMaxAbsX(p0, si, 3.0f);
        // The box appears.
        for (int f = 0; f < 30; ++f) si.frame(makeCam(0, 1.6f, 0.5f, 0, 27), withBox);
        CostMap c2; c2.build(si.map);
        PlanResult p1 = inc.plan(si.map, c2, ddx, ddy);
        Planner fresh; fresh.reset();
        PlanResult f1 = fresh.plan(si.map, c2, ddx, ddy);
        snprintf(buf, sizeof buf, "first plan full %d (%d expanded), straight |x| %.2f; box: incremental %d expanded (%d updated) vs full %d",
                 p0.fullReplan, p0.expanded, straight0, p1.expanded, p1.updated, f1.expanded);
        check("8a D* Lite: first plan is a full search, straight", p0.ok && p0.fullReplan && straight0 < 0.25f, buf);
        snprintf(buf, sizeof buf, "ok %d, clearance %.2f m, turn %.0f deg, fresh: turn %.0f deg, goal diff %.2f m",
                 p1.ok, p1.minClearance, p1.turnDeg, f1.turnDeg,
                 std::hypot(p1.goalX - f1.goalX, p1.goalY - f1.goalY));
        check("8b new obstacle on the path: re-planned around it", p1.ok && !p1.fullReplan &&
              p1.minClearance >= 0.22f && std::fabs(p1.turnDeg) > 5, buf);
        // Same answer as a full search (path length within 5 %, same goal area).
        snprintf(buf, sizeof buf, "length incremental %.2f m vs full %.2f m, goal diff %.2f m", p1.length, f1.length,
                 std::hypot(p1.goalX - f1.goalX, p1.goalY - f1.goalY));
        check("8c incremental result == full search result", f1.ok && std::fabs(p1.length - f1.length) <= 0.05f * f1.length + 0.05f &&
              std::hypot(p1.goalX - f1.goalX, p1.goalY - f1.goalY) < 0.3f, buf);
        // The user walks on (start moves every replan) past the box.
        int maxExp = 0, fullCount = 0; bool allOk = true; float minClr = 9;
        for (int k = 1; k <= 10; ++k) {
            for (int f = 0; f < 6; ++f) si.frame(makeCam(0, 1.6f, 0.5f + 0.05f * (6 * (k - 1) + f), 0, 27), withBox);
            CostMap ck; ck.build(si.map);
            PlanResult pk = inc.plan(si.map, ck, ddx, ddy);
            allOk = allOk && pk.ok; fullCount += pk.fullReplan; maxExp = std::max(maxExp, pk.expanded);
            minClr = std::min(minClr, pk.minClearance);
            if (getenv("DBG")) printf("   replan %d: expanded %d, updated %d, %.2f ms\n", k, pk.expanded, pk.updated, pk.ms);
        }
        snprintf(buf, sizeof buf, "10 replans while walking: all ok %d, full searches %d, max expanded %d, min clearance %.2f m",
                 allOk, fullCount, maxExp, minClr);
        check("8d walking on: incremental replans stay valid", allOk && fullCount == 0 && minClr >= 0.22f, buf);
    }
    // 9. Standing in front of a centred obstacle: the way round must not flip between left
    //    and right while the head moves a little (phone 17:15: "way is left" -> "way is
    //    right" within 3 s), but must switch when the chosen side gets blocked.
    {
        Scene c9;
        c9.boxes.push_back({-1.6f, 0, -1, -1.5f, 2.2f, 8, 1});
        c9.boxes.push_back({1.5f, 0, -1, 1.6f, 2.2f, 8, 1});
        c9.boxes.push_back({-0.3f, 0, 1.0f, 0.3f, 1.7f, 1.4f, 1});   // person-sized, dead centre
        Sim s9;
        for (int f = 0; f < 45; ++f) s9.frame(makeCam(0, 1.6f, 0.6f * f / 45, 0, 27), c9);
        Planner p9;
        int flips = 0, sides = 0, lastSide = 0, okCount = 0;
        for (int k = 0; k < 40; ++k) {
            const float yaw = 8.0f * std::sin(k * 0.9f);   // small head turns while standing
            const float sway = 0.03f * std::sin(k * 1.7f);   // body sway while standing
            for (int f = 0; f < 3; ++f) s9.frame(makeCam(sway, 1.6f, 0.6f, yaw, 27), c9);
            float dx, dy;
            worldDir(s9, std::sin(yaw / 57.2958f), std::cos(yaw / 57.2958f), dx, dy);
            CostMap c;
            c.build(s9.map);
            const PlanResult pr = p9.plan(s9.map, c, dx, dy);
            if (!pr.ok || std::fabs(pr.turnDeg) < 15.0f) continue;
            ++okCount;
            const int side = pathSideX(pr, s9, 1.2f);   // -1 = passes at x < 0, +1 = x > 0
            if (lastSide != 0 && side != lastSide) ++flips;
            lastSide = side;
            sides |= side > 0 ? 1 : 2;
            if (getenv("DBG")) printf("   k %d yaw %.1f sway %.2f turn %.0f side %d\n", k, yaw, sway, pr.turnDeg, side);
        }
        snprintf(buf, sizeof buf, "%d plans around the obstacle, %d side flips", okCount, flips);
        check("9a standing at an obstacle: the way keeps its side", okCount >= 20 && flips == 0, buf);
        // 9c. One blocked plan in between (phone: a noisy frame made the plan 'blocked'),
        //     then the same scene again: same goal, same side.
        {
            float dx0, dy0;
            worldDir(s9, 0, 1, dx0, dy0);
            CostMap c0;
            c0.build(s9.map);
            const PlanResult before = p9.plan(s9.map, c0, dx0, dy0);
            Scene wall = c9;
            wall.boxes.push_back({-1.5f, 0, 1.0f, 1.5f, 2.2f, 1.4f, 1});   // fully blocked
            Sim sw;
            for (int f = 0; f < 45; ++f) sw.frame(makeCam(0, 1.6f, 0.6f * f / 45, 0, 27), wall);
            float dxw, dyw;
            worldDir(sw, 0, 1, dxw, dyw);
            CostMap cw;
            cw.build(sw.map);
            const PlanResult blk = p9.plan(sw.map, cw, dxw, dyw);
            const PlanResult after = p9.plan(s9.map, c0, dx0, dy0);
            const float gd = std::hypot(after.goalX - before.goalX, after.goalY - before.goalY);
            snprintf(buf, sizeof buf, "in-between plan blocked %d; goal moved %.2f m, side %d -> %d", blk.blocked, gd,
                     pathSideX(before, s9, 1.2f), pathSideX(after, s9, 1.2f));
            check("9c a blocked plan in between keeps the goal and side",
                  blk.blocked && after.ok && gd < 0.3f && pathSideX(after, s9, 1.2f) == pathSideX(before, s9, 1.2f), buf);
        }
        // 9b. Block the chosen side: the plan must go round the other side.
        Scene b9 = c9;
        const float bx = lastSide > 0 ? 0.3f : -1.5f;   // fill the gap on the chosen side
        // Closes that side from 1 m away (nearer faces are inside the D455's minimum range or
        // hidden behind the centre obstacle).
        b9.boxes.push_back({bx, 0, 1.6f, bx + 1.2f, 1.7f, 2.6f, 1});
        for (int f = 0; f < 30; ++f) s9.frame(makeCam(0, 1.6f, 0.6f, 0, 27), b9);
        float dx, dy;
        worldDir(s9, 0, 1, dx, dy);
        CostMap cb;
        cb.build(s9.map);
        const PlanResult pb = p9.plan(s9.map, cb, dx, dy);
        const int sideB = pathSideX(pb, s9, 1.2f);
        snprintf(buf, sizeof buf, "chosen side x%s0 blocked: ok %d, turn %.0f deg, now x%s0", lastSide > 0 ? ">" : "<", pb.ok, pb.turnDeg, sideB > 0 ? ">" : "<");
        check("9b chosen side blocked: switches to the other side", pb.ok && std::fabs(pb.turnDeg) >= 15.0f && sideB == -lastSide, buf);
    }
    printf("plan failures: %d\n", fail);
    return fail;
}
