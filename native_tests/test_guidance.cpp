// Phase 5 guidance on synthetic plans: instruction choice, hysteresis, hold time.
#include <cmath>
#include <cstdio>

#include "guidance.h"

#include <map>

// Test input: what the validator receives (plus time). Each Guidance gets its own
// validator (the obstacle-ahead hysteresis lives there), as in native-lib.
struct TIn {
    double tSec{0};
    int safetyLevel{0}, safetyReason{0}, sensorLevel{0}, sensorReason{0};
    float aheadDist{-1};
    bool mapValid{false};
};
static GuideResult upd(Guidance& g, const LocalMap& m, const PlanResult& p, const TIn& t) {
    static std::map<Guidance*, SafetyValidator> validators;
    ValidatorInput v;
    v.safetyLevel = t.safetyLevel; v.safetyReason = t.safetyReason;
    v.sensorLevel = t.sensorLevel; v.sensorReason = t.sensorReason;
    v.aheadDist = t.aheadDist; v.mapValid = t.mapValid; v.tSec = t.tSec;
    GuideInput in; in.tSec = t.tSec; in.dec = validators[&g].validate(v, p);
    return g.update(m, p, in);
}

static int fails = 0;
static void check(const char* name, bool ok) { printf("  %-44s: %s\n", name, ok ? "OK" : "FAIL"); if (!ok) ++fails; }

// Path from (0,0) along +y for [straight] m, then bending by [bendDeg] for 2 m.
static PlanResult makePlan(float turnDeg, float straight, float bendDeg) {
    PlanResult p; p.ok = true; p.turnDeg = turnDeg;
    float x = 0, y = 0, a = 90.0f;
    p.path = {x, y};
    for (float d = 0.25f; d <= straight; d += 0.25f) { y += 0.25f; p.path.push_back(x); p.path.push_back(y); }
    a += bendDeg;
    for (int k = 0; k < 8; ++k) { x += 0.25f * std::cos(a / 57.2958f); y += 0.25f * std::sin(a / 57.2958f); p.path.push_back(x); p.path.push_back(y); }
    p.length = straight + 2;
    p.minClearance = 0.5f;
    p.nearClearance = 0.5f;
    return p;
}

static GuideResult run(Guidance& g, const LocalMap& m, const PlanResult& p, TIn in, double t0, double t1) {
    GuideResult r;
    for (double t = t0; t <= t1; t += 0.2) { in.tSec = t; r = upd(g, m, p, in); }
    return r;
}

int main() {
    LocalMap map;
    TIn in; in.mapValid = true;
    Guidance g;
    printf("guidance\n");
    GuideResult r = run(g, map, makePlan(0, 4, 0), in, 0, 1);
    check("straight path -> Go straight", r.code == GuideCode::Straight && r.text == "Go straight");
    const int s0 = r.seq;
    r = run(g, map, makePlan(20, 4, 0), in, 1.2, 2.2);
    check("20 deg -> Bear left", r.code == GuideCode::BearLeft && r.seq == s0 + 1);
    r = run(g, map, makePlan(-50, 4, 0), in, 2.4, 3.4);
    check("-50 deg -> Turn right", r.code == GuideCode::TurnRight);
    r = run(g, map, makePlan(-32, 4, 0), in, 3.6, 4.6);
    check("hysteresis: -32 deg keeps Turn right", r.code == GuideCode::TurnRight);
    r = run(g, map, makePlan(0, 2, 80), in, 4.8, 5.8);
    check("bend 80 deg at 2 m -> Turn left in 2 m", r.code == GuideCode::TurnLeftSoon && r.text == "Turn left in 2 m");
    // Jitter: alternating +-15 deg every 0.2 s must not flip the instruction.
    Guidance g2; run(g2, map, makePlan(0, 4, 0), in, 0, 1);
    const int sj = g2.current().seq; bool flipped = false;
    for (int k = 0; k < 20; ++k) { in.tSec = 1.2 + 0.2 * k; flipped |= upd(g2, map, makePlan(k % 2 ? 15 : -15, 4, 0), in).seq != sj; }
    check("+-15 deg jitter: no change", !flipped);
    // STOP is immediate.
    in.safetyLevel = 2; in.tSec = 6.0;
    r = upd(g, map, makePlan(0, 4, 0), in);
    check("safety STOP -> Stop at once", r.code == GuideCode::Stop && r.text == "Stop");
    // STOP keeps the way: path to the right -> "Stop, way is right" (after the hold).
    r = run(g, map, makePlan(-30, 4, 0), in, 6.2, 7.2);
    check("STOP + way right -> Stop, way is right", r.code == GuideCode::StopWayRight && r.text == "Stop, way is right");
    in.safetyLevel = 0;
    PlanResult b; b.blocked = true;
    // From a STOP, "No clear way" follows after the hold (no flicker).
    r = run(g, map, b, in, 7.4, 8.4);
    check("Stop -> No clear way after the hold", r.code == GuideCode::Blocked);
    Guidance gb; TIn ib; ib.mapValid = true;
    run(gb, map, makePlan(0, 4, 0), ib, 0, 1); ib.tSec = 1.05;
    check("Go straight -> No clear way at once", upd(gb, map, b, ib).code == GuideCode::Blocked);
    // Obstacle directly ahead within 1 m: STOP at once, over a good path; any type.
    Guidance g3; TIn i3; i3.mapValid = true;
    run(g3, map, makePlan(0, 4, 0), i3, 0, 1);
    i3.aheadDist = 0.45f; i3.tSec = 1.1;
    r = upd(g3, map, makePlan(0, 4, 0), i3);
    check("object at 0.45 m -> STOP! Obstacle directly ahead.", r.code == GuideCode::ObstacleAhead && r.text == "STOP! Obstacle directly ahead.");
    i3.aheadDist = 0.6f;
    r = run(g3, map, makePlan(0, 4, 0), i3, 1.2, 2.2);
    check("hysteresis: 0.6 m keeps the STOP", r.code == GuideCode::ObstacleAhead);
    i3.aheadDist = 0.45f; i3.safetyLevel = 2;
    // Object 0.45 m ahead and the planned way goes right: STOP, then the way around.
    Guidance g4; TIn i4 = i3; i4.tSec = 0;
    r = run(g4, map, makePlan(-30, 4, 0), i4, 0, 1);
    check("object at 0.45 m, way right -> Stop, way is right (re-path)", r.code == GuideCode::StopWayRight && r.pathVerified);
    // Path too close to obstacles: never suggest movement.
    Guidance g5; TIn i5; i5.mapValid = true;
    run(g5, map, makePlan(0, 4, 0), i5, 0, 1);
    PlanResult tight = makePlan(0, 4, 0); tight.minClearance = 0.15f; tight.nearClearance = 0.15f; i5.tSec = 1.1;
    r = upd(g5, map, tight, i5);
    check("unverified path -> Wait, at once", r.code == GuideCode::Unverified && r.text == "Wait, checking the way");
    i5.aheadDist = -1;
    r = run(g5, map, makePlan(0, 4, 0), i5, 1.2, 2.2);
    check("verified again -> Go straight", r.code == GuideCode::Straight && r.pathVerified);
    // Moving-object STOP flickering on / off while the plan is blocked: the
    // banner must not alternate Stop <-> No clear way every frame.
    Guidance g6; TIn i6; i6.mapValid = true;
    PlanResult bl; bl.blocked = true;
    run(g6, map, bl, i6, 0, 1);
    const int s6 = g6.current().seq; int changes = 0, last = s6;
    for (int k = 0; k < 20; ++k) {
        i6.tSec = 1.1 + 0.1 * k; i6.safetyLevel = (k % 2) ? 2 : 1;
        const int sq = upd(g6, map, bl, i6).seq; if (sq != last) { ++changes; last = sq; }
    }
    check("flickering STOP over blocked: <= 1 change", changes <= 1);
    // From a walking instruction, a safety STOP is still immediate.
    Guidance g7; TIn i7; i7.mapValid = true;
    run(g7, map, makePlan(0, 4, 0), i7, 0, 1);
    i7.safetyLevel = 2; i7.tSec = 1.05;
    check("Go straight -> STOP immediate", upd(g7, map, makePlan(0, 4, 0), i7).code == GuideCode::Stop);
    // Path goes right now (-30 deg) and turns left later: correct first.
    Guidance g8; TIn i8; i8.mapValid = true;
    r = run(g8, map, makePlan(-30, 2, 80), i8, 0, 1);
    check("right now + left later -> Bear right first", r.code == GuideCode::BearRight);
    r = run(g8, map, makePlan(-3, 2, 80), i8, 1.2, 2.2);
    check("... then Turn left in 2 m", r.code == GuideCode::TurnLeftSoon);
    // Sensor fault: STOP over everything, at once; degraded: no movement advice.
    Guidance g9; TIn i9; i9.mapValid = true;
    run(g9, map, makePlan(0, 4, 0), i9, 0, 1);
    i9.sensorLevel = 2; i9.sensorReason = 1; i9.aheadDist = 0.5f; i9.tSec = 1.05;
    r = upd(g9, map, makePlan(0, 4, 0), i9);
    check("camera blocked -> STOP! Camera blocked.", r.code == GuideCode::SensorFault && r.text == "STOP! Camera blocked.");
    i9.sensorLevel = 1; i9.sensorReason = 4; i9.aheadDist = -1;
    r = run(g9, map, makePlan(0, 4, 0), i9, 1.1, 3.0);
    check("degraded sensor -> Wait (no movement)", r.code == GuideCode::Unverified && !r.pathVerified);
    printf("guidance failures: %d\n", fails);
    return fails ? 1 : 0;
}
