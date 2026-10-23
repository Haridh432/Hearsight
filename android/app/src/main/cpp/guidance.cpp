#include "guidance.h"

#include <cmath>
#include <cstdio>

#include "nav_classes.h"

namespace {

bool isLeft(GuideCode c) { return c == GuideCode::BearLeft || c == GuideCode::TurnLeft || c == GuideCode::TurnLeftSoon; }
bool isStop(GuideCode c) { return c == GuideCode::Stop || c == GuideCode::ObstacleAhead || c == GuideCode::SensorFault || c == GuideCode::StopWayLeft || c == GuideCode::StopWayRight; }
bool isRight(GuideCode c) { return c == GuideCode::BearRight || c == GuideCode::TurnRight || c == GuideCode::TurnRightSoon; }

// Instructions that tell the user to walk somewhere.
bool isMove(GuideCode c) {
    return c == GuideCode::Straight || isLeft(c) || isRight(c) || c == GuideCode::DoorAhead;
}

std::string makeText(const GuideResult& r) {
    char b[64];
    const int d = static_cast<int>(std::lround(r.distance));
    switch (r.code) {
        case GuideCode::None: return "";
        case GuideCode::Straight: return "Go straight";
        case GuideCode::BearLeft: return "Bear left";
        case GuideCode::BearRight: return "Bear right";
        case GuideCode::TurnLeft: return "Turn left";
        case GuideCode::TurnRight: return "Turn right";
        case GuideCode::TurnLeftSoon: snprintf(b, sizeof b, "Turn left in %d m", d < 1 ? 1 : d); return b;
        case GuideCode::TurnRightSoon: snprintf(b, sizeof b, "Turn right in %d m", d < 1 ? 1 : d); return b;
        case GuideCode::DoorAhead: snprintf(b, sizeof b, "Door %d m ahead", d < 1 ? 1 : d); return b;
        case GuideCode::StairsAhead: return "Stairs ahead";
        case GuideCode::Stop: return "Stop";
        case GuideCode::Blocked: return "No clear way";
        case GuideCode::Arrived: return "Arrived";
        case GuideCode::StopWayLeft: return "Stop, way is left";
        case GuideCode::StopWayRight: return "Stop, way is right";
        case GuideCode::ObstacleAhead: return "STOP! Obstacle directly ahead.";
        case GuideCode::Unverified: return "Wait, checking the way";
        case GuideCode::SensorFault:
            // SensorFault reasons: 1 blocked, 2 frozen, 3 lag.
            return r.sensorReason == 1 ? "STOP! Camera blocked." : "STOP! Camera stalled.";
    }
    return "";
}

}  // namespace

const char* guideCodeName(GuideCode c) {
    static const char* n[] = {"none", "straight", "bear_left", "bear_right", "turn_left", "turn_right",
                              "turn_left_soon", "turn_right_soon", "door", "stairs", "stop", "blocked", "arrived", "stop_left", "stop_right", "obstacle_ahead", "unverified", "sensor_fault"};
    return n[static_cast<int>(c)];
}

GuideResult Guidance::decide(const LocalMap& map, const PlanResult& plan, const ValidatedDecision& dec) const {
    GuideResult r;
    r.pathVerified = dec.pathVerified;
    if (dec.verdict == Verdict::Halt) {
        if (dec.decidedBy == kCheckSensorHealth) {
            r.code = GuideCode::SensorFault;
            r.sensorReason = dec.sensorReason;
        } else if (dec.decidedBy == kCheckObstacleAhead && !(dec.pathVerified && std::fabs(plan.turnDeg) > bearDeg)) {
            r.code = GuideCode::ObstacleAhead;
            r.distance = dec.dist;
        } else {
            // Object ahead (or another STOP) with a verified way around: "Stop, way is left/right".
            // Keep the direction when the path is verified (hysteresis on the side).
            r.code = GuideCode::Stop;
            if (dec.pathVerified) {
                const bool was = cur_.code == GuideCode::StopWayLeft || cur_.code == GuideCode::StopWayRight;
                const float th = was ? bearDeg - hystDeg : bearDeg;
                r.angleDeg = plan.turnDeg;
                if (std::fabs(plan.turnDeg) > th) r.code = plan.turnDeg > 0 ? GuideCode::StopWayLeft : GuideCode::StopWayRight;
            }
        }
        return r;
    }
    if (dec.failed & kCheckStairs) { r.code = GuideCode::StairsAhead; return r; }
    if (!dec.mapValid) return r;
    if (plan.blocked) { r.code = GuideCode::Blocked; return r; }
    if (!plan.ok || plan.path.size() < 4) return r;
    if (!dec.mayMove) { r.code = GuideCode::Unverified; return r; }

    // Steering now: the planner's turn (relative to the smoothed walking
    // direction), with hysteresis around the current instruction.
    const float a = plan.turnDeg, aa = std::fabs(a);
    r.angleDeg = a;
    const bool wasTurn = cur_.code == GuideCode::TurnLeft || cur_.code == GuideCode::TurnRight;
    const bool wasSide = isLeft(cur_.code) || isRight(cur_.code);
    const float turnTh = wasTurn ? turnDeg - hystDeg : turnDeg + hystDeg;
    const float bearTh = wasSide ? bearDeg - hystDeg : bearDeg;
    if (aa > turnTh) { r.code = a > 0 ? GuideCode::TurnLeft : GuideCode::TurnRight; return r; }

    // Walk the path: first door cell on it, and the first point where the
    // path bends by more than turnDeg from its first stretch (>= 1 m ahead).
    float ox, oy;
    map.origin(ox, oy);
    const float cell = map.cellSize();
    const int n = map.cells();
    const auto& p = plan.path;
    const size_t m = p.size() / 2;
    float d = 0, doorD = -1, bendD = -1, bendA = 0;
    float h0x = 0, h0y = 0;
    for (size_t k = 1; k < m && d <= soonMaxDist; ++k) {
        const float sx = p[2 * k] - p[2 * k - 2], sy = p[2 * k + 1] - p[2 * k - 1];
        const float sl = std::hypot(sx, sy);
        if (sl < 1e-4f) continue;
        if (h0x == 0 && h0y == 0) { h0x = sx / sl; h0y = sy / sl; }
        d += sl;
        const int i = static_cast<int>((p[2 * k] - ox) / cell), j = static_cast<int>((p[2 * k + 1] - oy) / cell);
        if (doorD < 0 && i >= 0 && j >= 0 && i < n && j < n && map.semantic(i, j) == kNavDoor) doorD = d;
        if (bendD < 0 && d >= 1.0f) {
            const float cr = h0x * sy / sl - h0y * sx / sl, dt = h0x * sx / sl + h0y * sy / sl;
            const float b = std::atan2(cr, dt) * 57.2958f;
            if (std::fabs(b) > turnDeg) { bendD = d; bendA = b; }
        }
    }
    // Correct the current direction first; announce a later turn only when
    // already walking along the path (otherwise "Turn left in 1 m" could be
    // shown while the path goes right now).
    if (aa > bearTh) { r.code = a > 0 ? GuideCode::BearLeft : GuideCode::BearRight; return r; }
    if (bendD > 0) {
        r.code = bendA > 0 ? GuideCode::TurnLeftSoon : GuideCode::TurnRightSoon;
        r.distance = bendD;
        return r;
    }
    if (doorD > 0) { r.code = GuideCode::DoorAhead; r.distance = doorD; return r; }
    r.code = GuideCode::Straight;
    return r;
}

GuideResult Guidance::update(const LocalMap& map, const PlanResult& plan, const GuideInput& in) {
    GuideResult r = decide(map, plan, in.dec);
    // Defence in depth: never a walking instruction without the validator's permission.
    if (isMove(r.code) && !in.dec.mayMove) { r.code = GuideCode::Unverified; r.distance = 0; }
    // Immediate: an object directly ahead; and any change away from a
    // walking instruction (or from "Wait") to a stop-type one, so movement is
    // never suggested a moment too long. Changes between the stop-type
    // instructions (Stop / No clear way / which side) need the hold time, so
    // a flickering alert does not make them alternate.
    const bool fromMove = isMove(cur_.code) || cur_.code == GuideCode::None;
    const bool toHalt = !isMove(r.code);
    const bool urgent = (r.code == GuideCode::SensorFault && cur_.code != GuideCode::SensorFault) ||
                        (r.code == GuideCode::ObstacleAhead && cur_.code != GuideCode::ObstacleAhead &&
                         cur_.code != GuideCode::SensorFault) ||
                        (fromMove && toHalt && r.code != GuideCode::None) ||
                        (cur_.code == GuideCode::Unverified && isStop(r.code)) ||
                        (isMove(cur_.code) && !in.dec.mayMove);
    if (r.code == cur_.code) {
        // Same instruction: refresh angle / distance; the text (and seq) only
        // changes when the rounded distance does.
        pending_ = r.code;
        const std::string t = makeText(r);
        r.seq = cur_.seq;
        if (t != cur_.text && (std::fabs(r.distance - cur_.distance) > 0.6f || r.sensorReason != cur_.sensorReason))
            r.seq = ++seq_;
        else if (t != cur_.text) { r.distance = cur_.distance; }
        r.text = makeText(r);
        cur_ = r;
        return cur_;
    }
    if (!urgent) {
        if (r.code != pending_) { pending_ = r.code; pendingT_ = in.tSec; return cur_; }
        if (in.tSec - pendingT_ < holdSec) return cur_;
    }
    pending_ = r.code;
    r.text = makeText(r);
    r.seq = ++seq_;
    cur_ = r;
    return cur_;
}
