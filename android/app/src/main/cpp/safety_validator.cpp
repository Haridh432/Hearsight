#include "safety_validator.h"

#include <cmath>

#include "safety.h"
#include "sensor_health.h"

const char* safetyCheckName(uint32_t c) {
    switch (c) {
        case kCheckSensorHealth: return "sensor";
        case kCheckObstacleAhead: return "ahead";
        case kCheckDropOff: return "dropoff";
        case kCheckStairs: return "stairs";
        case kCheckHeadHeight: return "head";
        case kCheckObstacle: return "obstacle";
        case kCheckFloor: return "floor";
        case kCheckMoving: return "moving";
        case kCheckPathClearance: return "path";
        default: return "none";
    }
}

namespace {

uint32_t layer1Check(int reason) {
    switch (static_cast<SafetyReason>(reason)) {
        case SafetyReason::DropOff: return kCheckDropOff;
        case SafetyReason::Stairs: return kCheckStairs;
        case SafetyReason::HeadHeight: return kCheckHeadHeight;
        case SafetyReason::Obstacle: return kCheckObstacle;
        case SafetyReason::NoFloor:
        case SafetyReason::UnknownFloor:
        case SafetyReason::TripHazard: return kCheckFloor;
        default: return 0;
    }
}

const char* layer1Name(int reason) {
    switch (static_cast<SafetyReason>(reason)) {
        case SafetyReason::Obstacle: return "obstacle";
        case SafetyReason::HeadHeight: return "head";
        case SafetyReason::DropOff: return "dropoff";
        case SafetyReason::UnknownFloor: return "unknown";
        case SafetyReason::NoFloor: return "nofloor";
        case SafetyReason::Stairs: return "stairs";
        case SafetyReason::TripHazard: return "trip";
        default: return "none";
    }
}

}  // namespace

ValidatedDecision SafetyValidator::validate(const ValidatorInput& in, const PlanResult& plan) {
    ValidatedDecision d;
    d.sensorReason = in.sensorReason;
    d.mapValid = in.mapValid;

    // --- Every check, independently ---
    if (in.sensorLevel > 0) d.failed |= kCheckSensorHealth;
    // Object directly ahead: the ahead reading, or Layer 1's own obstacle in the walking
    // corridor (body / head height / trip) - phone test: the ahead reading dropped out for
    // single frames while Layer 1 still had the wall at 0.36 m.
    float ahead = in.aheadDist;
    if (in.safetyLevel > 0 && in.safetyDist > 0) {
        const auto r = static_cast<SafetyReason>(in.safetyReason);
        if ((r == SafetyReason::Obstacle || r == SafetyReason::HeadHeight || r == SafetyReason::TripHazard) &&
            (ahead < 0 || in.safetyDist < ahead))
            ahead = in.safetyDist;
    }
    // Release only after aheadHoldSec with nothing within clearAheadDist - whether the
    // reading is missing or far (phone test: an object seen on some frames only, a far
    // reading on the others, flipped the STOP every 0.1-0.2 s and let "Bear left" through).
    const bool nearNow = ahead > 0 && ahead <= clearAheadDist;
    if (nearNow) { aheadLastSeen_ = in.tSec; aheadLast_ = ahead; }
    const bool recent = aheadLastSeen_ >= 0 && in.tSec - aheadLastSeen_ < aheadHoldSec;
    // Not latched during the start-up warm-up (a first-frame reading must not hold a STOP).
    const bool warmup = in.sensorLevel == 1 && in.sensorReason == static_cast<int>(SensorFault::Warmup);
    if (ahead > 0 && ahead <= stopAheadDist && !warmup) aheadLatched_ = true;
    else if (aheadLatched_ && !nearNow && !recent) aheadLatched_ = false;
    if (aheadLatched_ && !nearNow) ahead = aheadLast_;   // report the last close distance
    const bool nearAhead = nearNow || recent;
    if (aheadLatched_) d.failed |= kCheckObstacleAhead;
    const uint32_t l1 = in.safetyLevel > 0 ? layer1Check(in.safetyReason) : 0;
    d.failed |= l1;
    // Moving objects: hold an alert for movingHoldSec (phone: a person at ~2 m flipped
    // clear / CAUTION / STOP every 30-70 ms, so banner and vibration flickered). A higher
    // level always takes over at once.
    int moving = in.movingLevel;
    float movingTtc = in.movingTtc;
    if (moving >= movingHeld_ && moving > 0) { movingHeld_ = moving; movingT_ = in.tSec; movingTtc_ = in.movingTtc; }
    else if (movingHeld_ > 0 && in.tSec - movingT_ < movingHoldSec) { moving = movingHeld_; movingTtc = movingTtc_; }
    else { movingHeld_ = moving; movingT_ = in.tSec; movingTtc_ = in.movingTtc; }
    if (moving > 0) d.failed |= kCheckMoving;
    // The path itself is verified on the map (clearance), whatever is close ahead; what is
    // close ahead limits the advice (detour only), below.
    d.pathVerified = in.mapValid && plan.ok && !plan.blocked && plan.path.size() >= 4 &&
                     plan.nearClearance >= verifyClearance - 1e-3f && in.sensorLevel == 0;
    if (!d.pathVerified) d.failed |= kCheckPathClearance;

    const auto r1 = static_cast<SafetyReason>(in.safetyReason);
    const bool objectReason = r1 == SafetyReason::Obstacle || r1 == SafetyReason::HeadHeight ||
                              r1 == SafetyReason::TripHazard;

    // --- Verdict, most severe first ---
    if (in.sensorLevel >= 2) {
        d.verdict = Verdict::Halt; d.decidedBy = kCheckSensorHealth;
    } else if (warmup) {
        // Start-up: "Wait, checking the way" (no advice, no alert) until the picture settles.
        d.verdict = Verdict::Caution; d.decidedBy = kCheckSensorHealth;
    } else if (aheadLatched_) {
        d.verdict = Verdict::Halt; d.decidedBy = kCheckObstacleAhead; d.dist = ahead;
    } else if (in.safetyLevel >= 2 && !(objectReason && in.safetyDist > stopAheadDist)) {
        d.verdict = Verdict::Halt; d.decidedBy = l1 ? l1 : kCheckObstacle; d.dist = in.safetyDist;
    } else if (in.safetyLevel >= 2) {
        // Layer 1 STOP for an object (obstacle / head height / trip) still beyond 0.5 m:
        // CAUTION, the user is guided around it (drop-off / stairs keep their STOP).
        d.verdict = Verdict::Caution; d.decidedBy = l1 ? l1 : kCheckObstacle; d.dist = in.safetyDist;
    } else if (moving >= 2) {
        d.verdict = Verdict::Halt; d.decidedBy = kCheckMoving; d.ttc = movingTtc;
    } else if (in.safetyLevel == 1) {
        d.verdict = Verdict::Caution; d.decidedBy = l1 ? l1 : kCheckObstacle; d.dist = in.safetyDist;
    } else if (moving == 1) {
        d.verdict = Verdict::Caution; d.decidedBy = kCheckMoving; d.ttc = movingTtc;
    } else if (in.sensorLevel == 1) {
        d.verdict = Verdict::Caution; d.decidedBy = kCheckSensorHealth;
    }

    // Walking advice only when nothing halts, the sensors are fully OK, no
    // stairs, and the path is verified.
    // Something close ahead (within clearAheadDist): only a detour that turns away from it.
    const bool detour = std::fabs(plan.turnDeg) >= detourTurnDeg;
    d.mayMove = d.verdict != Verdict::Halt && in.sensorLevel == 0 && !(l1 & kCheckStairs) && d.pathVerified &&
                (!nearAhead || detour);

    // --- Alert (banner + vibration) from the same verdict ---
    d.alertLevel = static_cast<int>(d.verdict);
    // Degraded sensors only withhold walking advice ("Wait"); no alert / vibration.
    if (d.verdict == Verdict::Caution && d.decidedBy == kCheckSensorHealth) d.alertLevel = 0;
    switch (d.decidedBy) {
        case 0: d.alertReason = "none"; break;
        case kCheckSensorHealth: d.alertReason = "sensor"; break;
        case kCheckObstacleAhead: d.alertReason = "ahead"; break;
        case kCheckMoving: d.alertReason = "moving"; break;
        default: d.alertReason = layer1Name(in.safetyReason); break;
    }
    return d;
}
