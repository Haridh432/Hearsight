// Safety validator: every check, priority order, hysteresis, and a randomized
// invariant test: walking advice only when every check passes.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "guidance.h"
#include "safety.h"
#include "safety_validator.h"
#include "sensor_health.h"

static int fails = 0;
static void check(const char* name, bool ok) { printf("  %-48s: %s\n", name, ok ? "OK" : "FAIL"); if (!ok) ++fails; }

static PlanResult goodPlan(float turn = 0, float clearance = 0.5f) {
    PlanResult p; p.ok = true; p.turnDeg = turn; p.minClearance = clearance; p.nearClearance = clearance; p.length = 4;
    for (int k = 0; k <= 16; ++k) { p.path.push_back(0); p.path.push_back(0.25f * k); }
    return p;
}

int main() {
    printf("safety validator\n");
    const PlanResult plan = goodPlan();
    ValidatorInput ok; ok.mapValid = true;
    ValidatedDecision d;
    {
        SafetyValidator v; d = v.validate(ok, plan);
        check("all clear + verified path -> GO, may move", d.verdict == Verdict::Go && d.mayMove && d.alertLevel == 0);
    }
    {
        SafetyValidator v; ValidatorInput i = ok; i.sensorLevel = 2; i.sensorReason = 1;
        d = v.validate(i, plan);
        check("sensor fault -> HALT (sensor)", d.verdict == Verdict::Halt && d.decidedBy == kCheckSensorHealth && !d.mayMove);
        i.aheadDist = 0.5f; d = v.validate(i, plan);
        check("sensor fault + object ahead -> sensor first", d.decidedBy == kCheckSensorHealth);
    }
    {
        // Start-up warm-up: an object reading on the first frames gives "Wait", not a held STOP.
        SafetyValidator v; ValidatorInput i = ok; i.sensorLevel = 1;
        i.sensorReason = static_cast<int>(SensorFault::Warmup); i.aheadDist = 0.33f;
        d = v.validate(i, plan);
        check("warm-up + object 0.33 m -> CAUTION, no alert", d.verdict == Verdict::Caution && !d.mayMove && d.alertLevel == 0);
        i.sensorLevel = 0; i.sensorReason = 0; i.aheadDist = -1; i.tSec = 0.1;
        d = v.validate(i, plan);
        check("after warm-up, nothing ahead -> no held STOP", d.verdict == Verdict::Go);
        i.aheadDist = 0.33f; i.tSec = 0.2; d = v.validate(i, plan);
        check("after warm-up, object 0.33 m -> HALT", d.verdict == Verdict::Halt && d.decidedBy == kCheckObstacleAhead);
    }
    {
        SafetyValidator v; ValidatorInput i = ok; i.aheadDist = 0.45f;
        d = v.validate(i, plan);
        check("object 0.45 m ahead -> HALT (ahead)", d.verdict == Verdict::Halt && d.decidedBy == kCheckObstacleAhead &&
              d.alertLevel == 2 && std::string(d.alertReason) == "ahead");
        i.aheadDist = 0.6f; d = v.validate(i, plan);
        check("then 0.6 m -> still HALT (hysteresis)", d.decidedBy == kCheckObstacleAhead);
        i.aheadDist = 0.9f; i.tSec = 0.2; d = v.validate(i, plan);
        check("then 0.9 m for 0.2 s -> still HALT (0.5 s clear needed)", d.decidedBy == kCheckObstacleAhead);
        i.tSec = 0.6; d = v.validate(i, plan);
        check("then 0.9 m for 0.6 s -> released, may move", d.verdict == Verdict::Go && d.mayMove);
        SafetyValidator v2; i.aheadDist = 0.6f; d = v2.validate(i, plan);
        check("fresh 0.6 m, straight path -> no HALT, no walking advice", d.verdict == Verdict::Go && !d.mayMove);
        SafetyValidator v3; d = v3.validate(i, goodPlan(40));
        check("fresh 0.6 m, path turns 40 deg away -> detour advice allowed", d.verdict == Verdict::Go && d.mayMove);
        SafetyValidator v4; ValidatorInput l1 = ok; l1.safetyLevel = 2;
        l1.safetyReason = static_cast<int>(SafetyReason::Obstacle); l1.safetyDist = 1.0f;
        d = v4.validate(l1, plan);
        check("Layer 1 obstacle STOP at 1.0 m -> CAUTION (only < 0.5 m stops)", d.verdict == Verdict::Caution && d.mayMove);
        l1.safetyReason = static_cast<int>(SafetyReason::DropOff);
        d = v4.validate(l1, plan);
        check("Layer 1 drop-off STOP at 1.0 m -> still HALT", d.verdict == Verdict::Halt && d.decidedBy == kCheckDropOff);
    }
    {
        // Phone test: the ahead reading dropped out for single frames while Layer 1 had the
        // wall at 0.36 m -> decision flipped ahead / obstacle and gave direction hints.
        SafetyValidator v; ValidatorInput i = ok; i.safetyLevel = 2;
        i.safetyReason = static_cast<int>(SafetyReason::Obstacle); i.safetyDist = 0.36f; i.aheadDist = -1;
        d = v.validate(i, plan);
        check("Layer 1 obstacle 0.36 m, no ahead reading -> HALT (ahead)", d.decidedBy == kCheckObstacleAhead &&
              !d.mayMove);
        SafetyValidator v2; ValidatorInput j = ok; j.aheadDist = 0.5f; j.tSec = 0;
        v2.validate(j, plan);
        j.aheadDist = -1; j.tSec = 0.2; d = v2.validate(j, plan);
        check("ahead reading drops out 0.2 s -> STOP holds", d.decidedBy == kCheckObstacleAhead);
        j.tSec = 0.8; d = v2.validate(j, plan);
        check("... gone for 0.8 s -> released", d.verdict == Verdict::Go);
        // Object seen on some frames only, a far reading on the others: no release.
        SafetyValidator v4; ValidatorInput f = ok; int released = 0;
        for (int n = 0; n < 30; ++n) {
            f.tSec = n * 0.033; f.aheadDist = (n % 3 == 0) ? 0.41f : 2.5f;
            d = v4.validate(f, plan);
            if (n > 0 && (d.verdict != Verdict::Halt || d.mayMove)) ++released;
        }
        check("0.51 m / 2.5 m flicker -> STOP holds, no advice", released == 0);
        f.aheadDist = 2.5f; f.tSec = 1.0 + 0.6; d = v4.validate(f, plan);
        check("... far for 0.6 s -> released", d.verdict == Verdict::Go && d.mayMove);
        ValidatorInput k = ok; k.safetyLevel = 2; k.safetyReason = static_cast<int>(SafetyReason::DropOff); k.safetyDist = 0.5f;
        SafetyValidator v3; d = v3.validate(k, plan);
        check("drop-off at 0.5 m is not 'object ahead'", d.decidedBy == kCheckDropOff);
    }
    {
        SafetyValidator v; ValidatorInput i = ok; i.safetyLevel = 2; i.safetyReason = static_cast<int>(SafetyReason::DropOff); i.safetyDist = 1.4f;
        d = v.validate(i, plan);
        check("Layer 1 drop-off STOP -> HALT (dropoff)", d.verdict == Verdict::Halt && d.decidedBy == kCheckDropOff &&
              std::string(d.alertReason) == "dropoff" && d.dist == 1.4f);
        check("... path still verified (direction hint allowed)", d.pathVerified && !d.mayMove);
        i.safetyReason = static_cast<int>(SafetyReason::HeadHeight);
        d = v.validate(i, plan);
        check("head-height STOP -> HALT (head)", d.decidedBy == kCheckHeadHeight);
    }
    {
        SafetyValidator v; ValidatorInput i = ok; i.safetyLevel = 1; i.safetyReason = static_cast<int>(SafetyReason::Stairs);
        d = v.validate(i, plan);
        check("stairs CAUTION -> no walking advice", d.verdict == Verdict::Caution && (d.failed & kCheckStairs) && !d.mayMove);
        i.safetyReason = static_cast<int>(SafetyReason::DropOff);
        d = v.validate(i, plan);
        check("drop-off CAUTION -> caution, may move on verified path", d.verdict == Verdict::Caution && d.mayMove);
    }
    {
        SafetyValidator v; ValidatorInput i = ok; i.movingLevel = 2; i.movingTtc = 1.5f;
        d = v.validate(i, plan);
        check("moving object STOP -> HALT (moving, ttc)", d.verdict == Verdict::Halt && d.decidedBy == kCheckMoving && d.ttc == 1.5f);
    }
    {
        // Phone: a person at ~2 m flipped clear / CAUTION / STOP every 30-70 ms.
        SafetyValidator v; ValidatorInput i = ok; int flips = 0; Verdict last = Verdict::Halt;
        const int pattern[] = {2, 0, 1, 0, 2, 0, 0, 1, 2, 0};
        for (int n = 0; n < 30; ++n) {
            i.tSec = n * 0.05; i.movingLevel = pattern[n % 10]; i.movingTtc = 1.5f;
            d = v.validate(i, plan);
            if (n > 0 && d.verdict != last) ++flips;
            last = d.verdict;
        }
        check("moving alert flicker -> steady HALT", flips == 0 && d.verdict == Verdict::Halt && d.decidedBy == kCheckMoving);
        i.movingLevel = 0; i.tSec = 1.45 + 0.3; d = v.validate(i, plan);
        check("... 0.3 s clear -> still held", d.verdict == Verdict::Halt);
        i.tSec = 1.45 + 0.6; d = v.validate(i, plan);
        check("... 0.6 s clear -> released", d.verdict == Verdict::Go && d.mayMove);
        SafetyValidator v2; ValidatorInput j = ok; j.movingLevel = 1; j.tSec = 0; v2.validate(j, plan);
        j.movingLevel = 2; j.tSec = 0.05; d = v2.validate(j, plan);
        check("CAUTION -> STOP takes over at once", d.verdict == Verdict::Halt);
    }
    {
        SafetyValidator v;
        // Phone: a pinch 5 m away (whole path 0.15 m) must not block advice for the next 3 m.
        PlanResult far = goodPlan(0, 0.15f); far.nearClearance = 0.4f;
        d = v.validate(ok, far);
        check("pinch only beyond 3 m -> still verified", d.pathVerified && d.mayMove);
        d = v.validate(ok, goodPlan(0, 0.224f));
        check("planner grid-step path (0.224 m) -> verified", d.pathVerified);
        d = v.validate(ok, goodPlan(0, 0.15f));
        check("path 0.15 m from obstacles -> not verified", !d.pathVerified && !d.mayMove && (d.failed & kCheckPathClearance));
        PlanResult b; b.blocked = true;
        d = v.validate(ok, b);
        check("blocked plan -> no walking advice", !d.mayMove);
        ValidatorInput nm = ok; nm.mapValid = false;
        d = v.validate(nm, plan);
        check("no map -> no walking advice", !d.mayMove);
    }
    {
        SafetyValidator v; ValidatorInput i = ok; i.sensorLevel = 1;
        d = v.validate(i, plan);
        check("sensor degraded -> caution, no alert, no advice", d.verdict == Verdict::Caution && d.alertLevel == 0 && !d.mayMove);
    }

    // Randomized: walking advice (validator and guidance) only when every check passes;
    // HALT always raises a STOP alert.
    {
        srand(7);
        SafetyValidator v; Guidance g; LocalMap map;
        int bad = 0, moves = 0;
        auto rnd = [](int n) { return rand() % n; };
        ValidatorInput i; PlanResult p;
        for (int k = 0; k < 100000; ++k) {
          if (k % 30 == 0) {   // a new random scenario every ~1 s (guidance holds changes 0.6 s)
            i = ValidatorInput();
            i.sensorLevel = rnd(10) == 0 ? rnd(3) : 0;
            i.sensorReason = rnd(4);
            i.safetyLevel = rnd(3);
            i.safetyReason = rnd(8);
            i.movingLevel = rnd(6) == 0 ? rnd(3) : 0;
            i.aheadDist = rnd(3) == 0 ? -1.0f : 0.3f + 0.01f * rnd(300);
            i.mapValid = rnd(10) != 0;
            p = rnd(5) == 0 ? PlanResult() : goodPlan(-60.0f + rnd(120), 0.05f * rnd(15));
            if (rnd(8) == 0) p.blocked = true;
            // Half the scenarios all clear, so walking advice is exercised.
            if (rnd(2) == 0) { i.sensorLevel = 0; i.safetyLevel = rnd(2) ? 0 : 1; i.movingLevel = 0; i.aheadDist = -1; i.mapValid = true; }
          }
          // Within a scenario, single checks flip on for one frame now and then.
          ValidatorInput f = i;
          if (rnd(20) == 0) f.sensorLevel = 2;
          if (rnd(20) == 0) f.aheadDist = 0.5f;
          if (rnd(20) == 0) f.movingLevel = 2;
          {
            const ValidatorInput& i = f;
            ValidatorInput ti = i; ti.tSec = k * 0.033;
            d = v.validate(ti, p);
            const bool aheadClose = i.aheadDist > 0 && i.aheadDist <= 0.7f && std::fabs(p.turnDeg) < 25.0f;
            const bool l1Halt = i.safetyLevel >= 2;   // safetyDist is unknown here -> a real STOP
            const bool allPass = i.sensorLevel == 0 && !l1Halt && i.movingLevel < 2 && !aheadClose &&
                                 !(i.safetyLevel > 0 && i.safetyReason == static_cast<int>(SafetyReason::Stairs)) &&
                                 i.mapValid && p.ok && !p.blocked && p.nearClearance >= 0.219f;
            if (d.mayMove && !allPass) ++bad;
            if (d.verdict == Verdict::Halt && d.alertLevel != 2) ++bad;
            GuideInput gi; gi.tSec = k * 0.033; gi.dec = d;
            const GuideCode c = g.update(map, p, gi).code;
            const bool move = c == GuideCode::Straight || c == GuideCode::BearLeft || c == GuideCode::BearRight ||
                              c == GuideCode::TurnLeft || c == GuideCode::TurnRight || c == GuideCode::TurnLeftSoon ||
                              c == GuideCode::TurnRightSoon || c == GuideCode::DoorAhead;
            if (move) { ++moves; if (!d.mayMove) ++bad; }
          }
        }
        printf("  (100000 random frames, %d walking instructions)\n", moves);
        check("random: never walking advice unless every check passes", bad == 0);
    }
    printf("validator failures: %d\n", fails);
    return fails ? 1 : 0;
}
