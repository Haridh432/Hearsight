#ifndef HSV2_SAFETY_VALIDATOR_H
#define HSV2_SAFETY_VALIDATOR_H

#include <cstdint>

#include "nav_plan.h"

/*
 * Safety validator: the single, deterministic authority on "may the user be
 * told to move?". Every check runs every frame; guidance only words the
 * result, and the alert (banner + vibration) comes from the same verdict, so
 * screen, voice and vibration can never disagree.
 *
 * Checks (bit in ValidatedDecision::failed):
 *   SensorHealth  camera picture not trustworthy (sensor_health.h)
 *   ObstacleAhead any object directly ahead within 0.5 m (released beyond 0.7 m)
 *   DropOff / Stairs / HeadHeight / Obstacle / Floor   Layer 1 safety (safety.h)
 *   Moving        approaching object (tracker time-to-collision)
 *   PathClearance planned path verified: not blocked, next 0.5-3 m >= 0.22 m from obstacles
 *
 * Verdict HALT on: sensor fault, obstacle ahead, any Layer 1 STOP, moving STOP.
 * mayMove (walking advice allowed) only when: no HALT, sensors not degraded,
 * no stairs, and the path is verified.
 */
enum class Verdict { Go = 0, Caution = 1, Halt = 2 };

enum SafetyCheck : uint32_t {
    kCheckSensorHealth = 1u << 0,
    kCheckObstacleAhead = 1u << 1,
    kCheckDropOff = 1u << 2,
    kCheckStairs = 1u << 3,
    kCheckHeadHeight = 1u << 4,
    kCheckObstacle = 1u << 5,
    kCheckFloor = 1u << 6,       // no floor / unknown floor / trip hazard
    kCheckMoving = 1u << 7,
    kCheckPathClearance = 1u << 8,
};

struct ValidatorInput {
    double tSec{0};          // frame time (s), for the dropout hold
    int sensorLevel{0};      // 0 ok, 1 degraded, 2 fault
    int sensorReason{0};     // SensorFault
    int safetyLevel{0};      // Layer 1: 0 clear, 1 caution, 2 stop
    int safetyReason{0};     // SafetyReason
    float safetyDist{-1};
    int movingLevel{0};      // tracker alert 0..2
    float movingTtc{-1};
    float aheadDist{-1};     // nearest object directly ahead, any type (-1 none)
    bool mapValid{false};
};

struct ValidatedDecision {
    Verdict verdict{Verdict::Go};
    uint32_t decidedBy{0};   // the check that set the verdict (one bit), 0 if Go
    uint32_t failed{0};      // every check not passing this frame
    bool pathVerified{false};// the planned path is verified safe (direction hints allowed)
    bool mayMove{false};     // walking advice allowed
    float dist{-1};          // distance for decidedBy (m), -1 if n/a
    float ttc{-1};           // moving objects (s)
    int sensorReason{0};
    bool mapValid{false};
    // Alert for the banner / vibration: 0 none, 1 caution, 2 stop, and its reason.
    int alertLevel{0};
    const char* alertReason{"none"};
};

class SafetyValidator {
public:
    // User: STOP only for something within 0.5 m ahead, then guide around it.
    float stopAheadDist{0.5f};
    float clearAheadDist{0.7f};
    // Within clearAheadDist, walking advice is allowed only along a path that turns
    // away from what is ahead (|turn| >= this), i.e. a detour, never "go straight".
    float detourTurnDeg{25.0f};
    // Path verified: the next 0.5-3 m keep at least the planner's body half-width
    // (CostMap::lethalRadius, 0.22 m) from obstacles. 0.25 m rejected the planner's own
    // grid-step paths (0.224 m) in every narrow passage.
    float verifyClearance{0.22f};
    double aheadHoldSec{0.5};   // a missing ahead reading keeps the STOP this long
    double movingHoldSec{0.5};  // a moving-object alert is held this long (tracker flicker)

    ValidatedDecision validate(const ValidatorInput& in, const PlanResult& plan);
    void reset() { aheadLatched_ = false; aheadLastSeen_ = -1; aheadLast_ = -1; movingHeld_ = 0; movingT_ = -1; movingTtc_ = -1; }

private:
    bool aheadLatched_{false};   // obstacle-ahead hysteresis (1.0 m on, 1.2 m off)
    double aheadLastSeen_{-1};   // last time a close (<= 1.2 m) object was seen ahead
    float aheadLast_{-1};
    int movingHeld_{0};          // moving-object alert level being held
    double movingT_{-1};         // when it was last seen at that level
    float movingTtc_{-1};
};

const char* safetyCheckName(uint32_t check);

#endif  // HSV2_SAFETY_VALIDATOR_H
