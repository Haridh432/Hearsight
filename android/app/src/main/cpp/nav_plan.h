#ifndef HSV2_NAV_PLAN_H
#define HSV2_NAV_PLAN_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "local_map.h"

/*
 * Phase 4b: cost map + human-centred path planning on the dynamic map.
 *
 * CostMap: distance (m) from every cell to the nearest obstacle / drop-off
 * (two-pass chamfer transform), turned into a walking cost:
 *   - lethal within lethalRadius (half body width + margin),
 *   - high inside the comfort zone, falling off with distance, so paths keep
 *     to the middle of corridors,
 *   - unknown cells cost extra (prefer what has been seen).
 *
 * Planner: Incremental D* Lite from the user's cell over the cost map (8-connected).
 * Goal: the cell furthest ahead (within +-goalConeDeg of the walking
 * direction, goalMin..goalMax m away) with the best progress-vs-cost trade;
 * the cell path is then string-pulled (shortcut while the straight segment
 * stays in low-cost cells) into a few waypoints.
 */
class CostMap {
public:
    void build(const LocalMap& map);
    int cells() const { return n_; }
    float distance(int i, int j) const { return dist_[static_cast<size_t>(j) * n_ + i]; }
    float cost(int i, int j) const { return cost_[static_cast<size_t>(j) * n_ + i]; }
    bool lethal(int i, int j) const { return cost(i, j) >= kLethal; }

    static constexpr float kLethal = 1e6f;
    // Half body width: a 0.6 m gap (narrow door) must stay passable; the
    // comfort cost still keeps paths away from walls in open space.
    float lethalRadius{0.22f};
    float comfortRadius{0.80f};
    float comfortWeight{6.0f};
    float unknownCost{2.0f};
    float occupiedProb{0.7f};

private:
    int n_{0};
    float cell_{0.1f};
    std::vector<float> dist_, cost_;
};

struct PlanResult {
    bool ok{false};
    std::vector<float> path;      // waypoints x0,y0,x1,y1,... (map plane m), from the user
    float goalX{0}, goalY{0};
    float length{0};              // m
    float turnDeg{0};             // direction to the first waypoint vs heading (+ = left)
    float minClearance{0};        // m, smallest obstacle distance along the path
    float nearClearance{0};       // m, the same over the next 0.5-3 m (what the user walks now)
    bool blocked{false};          // only a long detour / turning back exists: no clear way ahead
    double ms{0};
    // D* Lite statistics (this replan).
    int expanded{0};              // vertices expanded
    int updated{0};               // vertices whose edges / goal cost changed since the last replan
    bool fullReplan{false};       // search state rebuilt (first plan, map shift, size change)
};

/*
 * Local path planner: D* Lite (Koenig & Likhachev 2002) on the 8-connected
 * cost-map grid, re-planning incrementally between replans (5 Hz): only cells
 * whose cost changed (new obstacles, people, updated depth), the start (the
 * user moved) and the goal candidates are updated, instead of a full search.
 *
 * There is no single fixed goal: every free, seen cell 2.5-5 m ahead within the
 * walking cone is a candidate, connected to one virtual goal vertex with a
 * terminal cost (progress, clearance, straightness, previous-goal bonus). D*
 * Lite then finds the cheapest path from the user to the best candidate in one
 * search - the same choice the former Dijkstra + goal scan made.
 */
class Planner {
public:
    // [dirX, dirY]: walking direction (unit, map plane).
    PlanResult plan(const LocalMap& map, const CostMap& cost, float dirX, float dirY);

    float goalMin{2.5f}, goalMax{5.0f};
    float goalConeDeg{60.0f};
    float progressWeight{8.0f};   // cost units per metre of progress
    float smoothMaxCost{1.6f};    // string-pull freely through cells cheaper than this
    float goalClearance{0.6f};    // goal at least this far from obstacles
    // A path longer than detourFactor x straight distance + detourExtra, or
    // starting with a turn over maxTurnDeg, means "no clear way ahead".
    float detourFactor{1.8f}, detourExtra{1.5f}, maxTurnDeg{120.0f};
    // Stability between replans (5 Hz):
    float dirSmooth{0.3f};        // walking direction low-pass (weight of the new value)
    float straightWeight{6.0f};   // cost for goals away from the walking direction
    float keepGoalMargin{10.0f};   // keep the previous goal unless a new one is this much better
    // The goal is remembered through this many consecutive blocked plans (~2 s at 5 Hz):
    // phone 17:15, one noisy blocked plan erased it and the next plan went round the other
    // side ("way is left" -> "way is right" within 3 s). Reused only while still valid.
    int keepGoalBlockedPlans{10};
    int smoothIters{2};           // corner rounding (Chaikin) passes
    void reset() { haveGoal_ = false; haveDir_ = false; dsInit_ = false; blockedStreak_ = 0; }
    // Drops the incremental search state only (next plan = full search); for checks.
    void invalidateSearch() { dsInit_ = false; }

private:
    bool haveGoal_{false}, haveDir_{false};
    int blockedStreak_{0};
    void forgetGoalIfBlocked() { if (++blockedStreak_ > keepGoalBlockedPlans) haveGoal_ = false; }
    float goalX_{0}, goalY_{0};
    float dirX_{0}, dirY_{1};

    // --- D* Lite state (cells 0..N-1 row-major, N = virtual goal) ---
    struct Key { float k1, k2; };
    struct QItem { float k1, k2; int v; };
    struct QCmp { bool operator()(const QItem& a, const QItem& b) const {
        return a.k1 > b.k1 || (a.k1 == b.k1 && a.k2 > b.k2); } };
    bool dsInit_{false};
    int dsN_{0};                  // grid size (cells per side)
    float dsOx_{0}, dsOy_{0};     // map origin the state belongs to
    float dsCell_{0.1f};
    int dsStart_{-1}, dsLast_{-1};
    float km_{0};
    std::vector<float> g_, rhs_, cost_, term_;
    std::vector<Key> key_;
    std::vector<char> open_;
    std::vector<QItem> heap_;
    int expanded_{0};
    float edgeCost(int u, int v) const;     // u -> v (entering v), neighbours
    float heur(int a, int b) const;         // octile distance x cell (admissible: cost >= 1)
    Key calcKey(int v) const;
    void updateVertex(int u);
    void computeShortestPath(int maxExpand);
    void push(int v);
    float cellCost(int v) const;            // cost of entering cell v (escape near start)
};

#endif  // HSV2_NAV_PLAN_H
