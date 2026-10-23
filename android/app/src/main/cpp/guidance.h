#ifndef HSV2_GUIDANCE_H
#define HSV2_GUIDANCE_H

#include <string>

#include "local_map.h"
#include "nav_plan.h"
#include "safety_validator.h"

/*
 * Phase 5 guidance: turns the planned path (+ safety state) into one short,
 * stable walking instruction, e.g. "Go straight", "Bear left",
 * "Turn right in 2 m", "Door ahead, 3 m", "Stop", "No clear way".
 *
 * Stability: a new instruction must hold for [holdSec] before it replaces the
 * current one (STOP / blocked switch immediately), and the angle thresholds
 * have hysteresis, so a swaying head does not flip left / right. [seq]
 * increments whenever the spoken instruction changes (Phase 6 speaks it).
 */
enum class GuideCode {
    None = 0,     // no map / plan yet
    Straight,
    BearLeft, BearRight,     // small correction
    TurnLeft, TurnRight,     // the path turns now (> turnDeg)
    TurnLeftSoon, TurnRightSoon,  // a turn further along the path
    DoorAhead,
    StairsAhead,
    Stop,         // safety STOP
    Blocked,      // planner: no clear way ahead
    Arrived,      // (reserved for a set destination)
    StopWayLeft,  // safety STOP, the planned way continues to the left
    StopWayRight,
    ObstacleAhead, // any object directly ahead within stopAheadDist: "STOP! Obstacle directly ahead."
    Unverified,    // a path exists but is not verified safe: no movement is suggested
    SensorFault    // the camera picture cannot be trusted: STOP
};

struct GuideResult {
    GuideCode code{GuideCode::None};
    float angleDeg{0};   // steering angle now (+ = left)
    float distance{0};   // m to the upcoming turn / door / stairs / obstacle
    bool pathVerified{false};  // movement may be suggested (green path)
    int sensorReason{0};       // SensorFault reason (for the text)
    int seq{0};          // increments when [text] changes
    std::string text;
};

// Guidance words the safety validator's decision; it never decides safety itself.
struct GuideInput {
    double tSec{0};
    ValidatedDecision dec;
};

class Guidance {
public:
    float bearDeg{12};       // |angle| above this: bear
    float turnDeg{35};       // |angle| above this: turn
    float hystDeg{5};
    float soonMaxDist{3.0f}; // announce turns / doors within this distance
    float holdSec{0.6f};

    GuideResult update(const LocalMap& map, const PlanResult& plan, const GuideInput& in);
    void reset() { cur_ = GuideResult(); pending_ = GuideCode::None; pendingT_ = -1; seq_ = 0; }
    const GuideResult& current() const { return cur_; }

private:
    GuideResult decide(const LocalMap& map, const PlanResult& plan, const ValidatedDecision& dec) const;
    GuideResult cur_;
    GuideCode pending_{GuideCode::None};
    double pendingT_{-1};
    int seq_{0};
};

const char* guideCodeName(GuideCode c);

#endif  // HSV2_GUIDANCE_H
