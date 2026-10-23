#include "nav_plan.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>

void CostMap::build(const LocalMap& map) {
    n_ = map.cells();
    cell_ = map.cellSize();
    const size_t N = static_cast<size_t>(n_) * n_;
    const float big = 1e4f;
    dist_.assign(N, big);
    // Seeds: occupied or drop-off cells.
    for (int j = 0; j < n_; ++j)
        for (int i = 0; i < n_; ++i)
            if (map.occupancy(i, j) > occupiedProb || map.hazard(i, j)) dist_[static_cast<size_t>(j) * n_ + i] = 0;
    // Two-pass chamfer (1, sqrt2) in cells.
    const float d1 = 1.0f, d2 = 1.4142f;
    for (int j = 0; j < n_; ++j)
        for (int i = 0; i < n_; ++i) {
            float& d = dist_[static_cast<size_t>(j) * n_ + i];
            if (i > 0) d = std::min(d, dist_[static_cast<size_t>(j) * n_ + i - 1] + d1);
            if (j > 0) {
                d = std::min(d, dist_[static_cast<size_t>(j - 1) * n_ + i] + d1);
                if (i > 0) d = std::min(d, dist_[static_cast<size_t>(j - 1) * n_ + i - 1] + d2);
                if (i < n_ - 1) d = std::min(d, dist_[static_cast<size_t>(j - 1) * n_ + i + 1] + d2);
            }
        }
    for (int j = n_ - 1; j >= 0; --j)
        for (int i = n_ - 1; i >= 0; --i) {
            float& d = dist_[static_cast<size_t>(j) * n_ + i];
            if (i < n_ - 1) d = std::min(d, dist_[static_cast<size_t>(j) * n_ + i + 1] + d1);
            if (j < n_ - 1) {
                d = std::min(d, dist_[static_cast<size_t>(j + 1) * n_ + i] + d1);
                if (i < n_ - 1) d = std::min(d, dist_[static_cast<size_t>(j + 1) * n_ + i + 1] + d2);
                if (i > 0) d = std::min(d, dist_[static_cast<size_t>(j + 1) * n_ + i - 1] + d2);
            }
        }
    cost_.assign(N, 1.0f);
    for (int j = 0; j < n_; ++j)
        for (int i = 0; i < n_; ++i) {
            const size_t k = static_cast<size_t>(j) * n_ + i;
            dist_[k] *= cell_;   // metres
            const float d = dist_[k];
            float c;
            if (d < lethalRadius) c = kLethal;
            else if (d < comfortRadius)
                c = 1.0f + comfortWeight * std::pow((comfortRadius - d) / (comfortRadius - lethalRadius), 2.0f);
            else c = 1.0f;
            if (c < kLethal && !map.known(i, j)) c += unknownCost;
            cost_[k] = c;
        }
}

namespace {
constexpr float kInf = 1e30f;
}

float Planner::cellCost(int v) const { return cost_[v]; }

float Planner::edgeCost(int u, int v) const {
    const float c = cost_[v];
    if (c >= kInf) return kInf;
    const int du = std::abs(u % dsN_ - v % dsN_), dv = std::abs(u / dsN_ - v / dsN_);
    return ((du + dv == 2) ? 1.4142f : 1.0f) * dsCell_ * c;
}

float Planner::heur(int a, int b) const {
    if (a >= dsN_ * dsN_ || b >= dsN_ * dsN_) return 0;   // the virtual goal
    const float dx = static_cast<float>(std::abs(a % dsN_ - b % dsN_));
    const float dy = static_cast<float>(std::abs(a / dsN_ - b / dsN_));
    return dsCell_ * (std::max(dx, dy) + 0.4142f * std::min(dx, dy));   // octile, cost >= 1 per metre
}

Planner::Key Planner::calcKey(int v) const {
    const float m = std::min(g_[v], rhs_[v]);
    if (m >= kInf) return {kInf, kInf};
    return {m + heur(dsStart_, v) + km_, m};
}

void Planner::push(int v) {
    const Key k = calcKey(v);
    key_[v] = k;
    open_[v] = 1;
    heap_.push_back({k.k1, k.k2, v});
    std::push_heap(heap_.begin(), heap_.end(), QCmp());
}

void Planner::updateVertex(int u) {
    const int N = dsN_ * dsN_;
    if (u != N) {
        // rhs(u) = min over successors (neighbours entered at their cost, + the
        // virtual goal through u's terminal cost).
        float best = term_[u] < kInf ? term_[u] + g_[N] : kInf;
        const int ui = u % dsN_, uj = u / dsN_;
        for (int dj = -1; dj <= 1; ++dj)
            for (int di = -1; di <= 1; ++di) {
                if (!di && !dj) continue;
                const int ni = ui + di, nj = uj + dj;
                if (ni < 0 || nj < 0 || ni >= dsN_ || nj >= dsN_) continue;
                const int v = nj * dsN_ + ni;
                if (g_[v] >= kInf) continue;
                const float c = edgeCost(u, v);
                if (c < kInf) best = std::min(best, c + g_[v]);
            }
        rhs_[u] = best;
    }
    if (g_[u] != rhs_[u]) push(u);
    else open_[u] = 0;
}

void Planner::computeShortestPath(int maxExpand) {
    const int N = dsN_ * dsN_;
    auto less = [](const Key& a, const Key& b) { return a.k1 < b.k1 || (a.k1 == b.k1 && a.k2 < b.k2); };
    while (!heap_.empty() && expanded_ < maxExpand) {
        const QItem top = heap_.front();
        const int u = top.v;
        // Lazy deletion: skip entries that are no longer current.
        if (!open_[u] || top.k1 != key_[u].k1 || top.k2 != key_[u].k2) {
            std::pop_heap(heap_.begin(), heap_.end(), QCmp());
            heap_.pop_back();
            continue;
        }
        const Key ks = calcKey(dsStart_);
        // Stop once the top key is clearly ABOVE the start's. Ties are expanded too, with a
        // float tolerance: a vertex on the path whose key equals the start's up to rounding
        // (h == edge cost on cost-1 cells) and left unexpanded kept a stale, too low g and
        // the path walk looped.
        if (top.k1 > ks.k1 + 0.01f && rhs_[dsStart_] <= g_[dsStart_]) break;
        std::pop_heap(heap_.begin(), heap_.end(), QCmp());
        heap_.pop_back();
        ++expanded_;
        const Key kNew = calcKey(u);
        if (less(Key{top.k1, top.k2}, kNew)) { push(u); continue; }   // key grew (km): re-queue
        open_[u] = 0;
        auto forPreds = [&](auto fn) {
            if (u == N) {   // the virtual goal: its predecessors are the candidates
                for (int v = 0; v < N; ++v) if (term_[v] < kInf) fn(v);
                return;
            }
            const int ui = u % dsN_, uj = u / dsN_;
            for (int dj = -1; dj <= 1; ++dj)
                for (int di = -1; di <= 1; ++di) {
                    if (!di && !dj) continue;
                    const int ni = ui + di, nj = uj + dj;
                    if (ni >= 0 && nj >= 0 && ni < dsN_ && nj < dsN_) fn(nj * dsN_ + ni);
                }
        };
        // Optimised D* Lite updates (Koenig & Likhachev, Fig. 4): a lowered g only
        // lowers predecessors' rhs (O(1) each); a raised g recomputes only the
        // predecessors whose rhs came through u.
        auto cPred = [&](int p) { return u == N ? term_[p] : edgeCost(p, u); };
        auto queueUpd = [&](int p) { if (g_[p] != rhs_[p]) push(p); else open_[p] = 0; };
        if (g_[u] > rhs_[u]) {
            g_[u] = rhs_[u];
            forPreds([&](int p) {
                const float c = cPred(p);
                if (c < kInf && c + g_[u] < rhs_[p]) { rhs_[p] = c + g_[u]; queueUpd(p); }
            });
        } else {
            const float gOld = g_[u];
            g_[u] = kInf;
            forPreds([&](int p) {
                const float c = cPred(p);
                if (c < kInf && rhs_[p] == c + gOld) updateVertex(p);
            });
            if (u != N) updateVertex(u);
            else queueUpd(u);
        }
    }
}

PlanResult Planner::plan(const LocalMap& map, const CostMap& cm, float dirX, float dirY) {
    const auto t0 = std::chrono::steady_clock::now();
    PlanResult r;
    const int n = cm.cells();
    const float cell = map.cellSize();
    float ux, uy, hx, hy, ox, oy;
    map.user(ux, uy, hx, hy);
    map.origin(ox, oy);
    auto toCell = [&](float x, float y, int& i, int& j) {
        i = static_cast<int>(std::floor((x - ox) / cell));
        j = static_cast<int>(std::floor((y - oy) / cell));
        return i >= 0 && j >= 0 && i < n && j < n;
    };
    int si, sj;
    if (!toCell(ux, uy, si, sj)) return r;

    // Smoothed walking direction: head turns / nods must not swing the goal.
    if (!haveDir_) { dirX_ = dirX; dirY_ = dirY; haveDir_ = true; }
    else {
        dirX_ = (1 - dirSmooth) * dirX_ + dirSmooth * dirX;
        dirY_ = (1 - dirSmooth) * dirY_ + dirSmooth * dirY;
    }
    {
        const float l = std::hypot(dirX_, dirY_);
        if (l > 1e-3f) { dirX_ /= l; dirY_ /= l; } else { dirX_ = dirX; dirY_ = dirY; }
    }
    dirX = dirX_; dirY = dirY_;

    // --- D* Lite (incremental) ---
    const int N = n * n;
    const int G = N;   // virtual goal vertex
    const int s = sj * n + si;
    const bool reinit = !dsInit_ || dsN_ != n || std::fabs(dsOx_ - ox) > 1e-4f || std::fabs(dsOy_ - oy) > 1e-4f ||
                        std::fabs(dsCell_ - cell) > 1e-6f;
    r.fullReplan = reinit;
    if (reinit) {
        dsInit_ = true; dsN_ = n; dsOx_ = ox; dsOy_ = oy; dsCell_ = cell;
        g_.assign(N + 1, kInf); rhs_.assign(N + 1, kInf);
        cost_.assign(N, -1.0f); term_.assign(N, kInf);
        key_.assign(N + 1, Key{kInf, kInf}); open_.assign(N + 1, 0);
        heap_.clear();
        km_ = 0;
        dsStart_ = dsLast_ = s;
    }
    // The user moved: shift the key modifier (keys stay lower bounds).
    dsStart_ = s;
    if (dsLast_ != s) { km_ += heur(dsLast_, s); dsLast_ = s; }
    expanded_ = 0;

    const float cosCone = std::cos(goalConeDeg / 57.2957795f);
    const float offset = progressWeight * (goalMax + 1.0f) + keepGoalMargin + 1.0f;
    int prevGoal = -1;
    if (haveGoal_) {
        int pi, pj;
        if (toCell(goalX_, goalY_, pi, pj)) prevGoal = pj * n + pi;
    }
    // Goal candidates are scored from the user's current position and walking
    // direction every replan (a candidate set held back for 0.5 m / 10 deg, to save
    // updates, picked stale goals: U-turns rejected as detours, more "no clear way").
    const float tax = ux, tay = uy, tdx = dirX, tdy = dirY;
    // Search only around the user (goals are <= goalMax away): cells farther than
    // goalMax + 1 m are outside the graph. Without this the search wandered over the
    // whole map (slow, long detours rejected as "blocked").
    const float searchR = goalMax + 1.0f;
    const float searchR2 = searchR * searchR;
    // Effective cell costs: lethal = impassable, except within 0.5 m of the start
    // (the user may stand inside the inflated zone; it can be left, at a price).
    std::vector<int> affected;
    affected.reserve(1024);
    std::vector<char> mark(N + 1, 0);
    auto touch = [&](int v) { if (!mark[v]) { mark[v] = 1; affected.push_back(v); } };
    const int di[8] = {1, -1, 0, 0, 1, 1, -1, -1}, dj[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            const int v = j * n + i;
            float c = cm.cost(i, j);
            {
                const float ax = ox + (i + 0.5f) * cell - tax, ay = oy + (j + 0.5f) * cell - tay;
                if (ax * ax + ay * ay > searchR2) c = CostMap::kLethal * 2;   // outside the search area
            }
            if (c >= CostMap::kLethal * 2) c = kInf;
            else if (c >= CostMap::kLethal) {
                const float dx = (i - si) * cell, dy = (j - sj) * cell;
                c = (dx * dx + dy * dy <= 0.25f) ? 20.0f : kInf;
            }
            if (c != cost_[v]) {
                cost_[v] = c;
                // Edges INTO v changed: all neighbours' rhs may change.
                for (int q = 0; q < 8; ++q) {
                    const int ni = i + di[q], nj = j + dj[q];
                    if (ni >= 0 && nj >= 0 && ni < n && nj < n) touch(nj * n + ni);
                }
            }
        }

    // Goal candidates: terminal cost to the virtual goal (the former goal score
    // without the path cost; >= 0 by the offset). The previous goal gets a bonus
    // of keepGoalMargin (kept unless a new one is clearly better).
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            const int v = j * n + i;
            float t = kInf;
            if (cost_[v] < kInf && cm.cost(i, j) < CostMap::kLethal && map.known(i, j)) {
                const float x = ox + (i + 0.5f) * cell - tax, y = oy + (j + 0.5f) * cell - tay;
                const float d = std::sqrt(x * x + y * y);
                const float along = x * tdx + y * tdy;
                const bool isPrev = v == prevGoal && d > 1.0f && d < goalMax + 0.5f && along > d * cosCone;
                if (isPrev || (d >= goalMin && d <= goalMax && along >= d * cosCone)) {
                    // Prefer goals clear of walls (penalty, not a hard rule: behind an
                    // obstacle the clear centre may be unseen) and straight ahead.
                    const float dc = cm.distance(i, j);
                    t = offset - progressWeight * along + (dc < goalClearance ? 40.0f * (goalClearance - dc) : 0.0f) +
                        straightWeight * (1.0f - along / d) - (isPrev ? keepGoalMargin : 0.0f);
                }
            }
            if (t != term_[v]) { term_[v] = t; touch(v); }
        }
    if (reinit) {
        rhs_[G] = 0;
        push(G);
    }
    r.updated = static_cast<int>(affected.size());
    for (int v : affected) updateVertex(v);
    computeShortestPath(4 * N + 100);
    r.expanded = expanded_;

    if (g_[s] >= kInf * 0.5f && rhs_[s] >= kInf * 0.5f) {
        r.blocked = true;   // nothing reachable ahead
        forgetGoalIfBlocked();
        r.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        return r;
    }
    // Path: follow the best successor (cost + g) from the user until the virtual goal.
    std::vector<int> cells;
    int goal = -1;
    auto extract = [&]() {
        cells.assign(1, s);
        goal = -1;
        std::vector<char> seen(N, 0);
        seen[s] = 1;
        for (int cur = s, steps = 0; steps < N; ++steps) {
            float best = term_[cur] < kInf ? term_[cur] + g_[G] : kInf;
            int next = G;
            const int ci = cur % n, cj = cur / n;
            for (int q = 0; q < 8; ++q) {
                const int ni = ci + di[q], nj = cj + dj[q];
                if (ni < 0 || nj < 0 || ni >= n || nj >= n) continue;
                const int v = nj * n + ni;
                const float c = edgeCost(cur, v);
                if (c >= kInf || g_[v] >= kInf) continue;
                if (c + g_[v] < best) { best = c + g_[v]; next = v; }
            }
            if (best >= kInf) return;
            if (next == G) { goal = cur; return; }
            if (seen[next]) return;   // loop: inconsistent state
            seen[next] = 1;
            cur = next;
            cells.push_back(cur);
        }
    };
    extract();
    if (goal < 0 && !reinit) {
        // Safety net: rebuild the search from scratch (a bookkeeping slip must never
        // turn into a false "no clear way").
        dsInit_ = false;
        PlanResult full = plan(map, cm, dirX, dirY);
        full.fullReplan = true;
        full.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        return full;
    }
    if (goal < 0) {
        r.blocked = true;
        forgetGoalIfBlocked();
        r.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        return r;
    }
    auto cx = [&](int k) { return ox + (k % n + 0.5f) * cell; };
    auto cy = [&](int k) { return oy + (k / n + 0.5f) * cell; };

    // String pulling: from the current anchor, jump to the furthest path cell
    // reachable in a straight line through cheap cells.
    // A shortcut may not be costlier than the stretch of path it replaces
    // (so a path squeezing through a narrow gap can still be straightened).
    auto clearLine = [&](float x0, float y0, float x1, float y1, float limit) {
        const float len = std::hypot(x1 - x0, y1 - y0);
        const int steps = std::max(1, static_cast<int>(len / (cell * 0.5f)));
        for (int t = 1; t <= steps; ++t) {
            const float x = x0 + (x1 - x0) * t / steps, y = y0 + (y1 - y0) * t / steps;
            int i, j;
            if (!toCell(x, y, i, j)) return false;
            const float c = cm.cost(i, j);
            if (c >= CostMap::kLethal || c > limit) return false;
        }
        return true;
    };
    std::vector<float> wp = {ux, uy};
    size_t a = 0;
    while (a + 1 < cells.size()) {
        size_t bestB = a + 1;
        const float ax = (a == 0) ? ux : cx(cells[a]), ay = (a == 0) ? uy : cy(cells[a]);
        // Running max of the path cost from a to each b.
        std::vector<float> runMax(cells.size(), 0);
        float m = 0;
        for (size_t b = a + 1; b < cells.size(); ++b) {
            const float c = cm.cost(cells[b] % n, cells[b] / n);
            if (c < CostMap::kLethal) m = std::max(m, c);
            runMax[b] = m;
        }
        for (size_t b = cells.size() - 1; b > a + 1; --b)
            if (clearLine(ax, ay, cx(cells[b]), cy(cells[b]), std::max(smoothMaxCost, runMax[b]) + 0.05f)) {
                bestB = b;
                break;
            }
        wp.push_back(cx(cells[bestB]));
        wp.push_back(cy(cells[bestB]));
        a = bestB;
    }
    // Corner rounding (Chaikin), kept only where it stays out of lethal cells.
    for (int it = 0; it < smoothIters && wp.size() >= 6; ++it) {
        std::vector<float> sm = {wp[0], wp[1]};
        for (size_t i = 0; i + 3 < wp.size(); i += 2) {
            const float ax = wp[i], ay = wp[i + 1], bx = wp[i + 2], by = wp[i + 3];
            sm.push_back(0.75f * ax + 0.25f * bx); sm.push_back(0.75f * ay + 0.25f * by);
            sm.push_back(0.25f * ax + 0.75f * bx); sm.push_back(0.25f * ay + 0.75f * by);
        }
        sm.push_back(wp[wp.size() - 2]); sm.push_back(wp.back());
        bool safe = true;
        for (size_t i = 0; i + 1 < sm.size() && safe; i += 2) {
            int ci, cj;
            if (toCell(sm[i], sm[i + 1], ci, cj) && cm.lethal(ci, cj) &&
                std::hypot(sm[i] - ux, sm[i + 1] - uy) > 0.5f) safe = false;
        }
        if (!safe) break;
        wp.swap(sm);
    }
    r.path = wp;
    r.ok = wp.size() >= 4;
    r.goalX = cx(goal);
    r.goalY = cy(goal);
    goalX_ = r.goalX; goalY_ = r.goalY; haveGoal_ = true;
    blockedStreak_ = 0;
    // Clearance along the path, not counting the first 0.5 m (the user may
    // stand inside the inflated zone and the path first leads out of it).
    // nearClearance: the same over 0.5-3 m only - what the user is about to walk; the far
    // part is re-planned many times on the way (phone: one pinch 5 m away made the whole
    // path "unverified" -> "Wait" with free floor ahead).
    r.minClearance = 1e9f;
    r.nearClearance = 1e9f;
    for (int k : cells) {
        const float x = cx(k) - ux, y = cy(k) - uy;
        const float d2 = x * x + y * y;
        if (d2 < 0.25f) continue;
        const float c = cm.distance(k % n, k / n);
        r.minClearance = std::min(r.minClearance, c);
        if (d2 <= 9.0f) r.nearClearance = std::min(r.nearClearance, c);
    }
    if (r.minClearance > 1e8f) r.minClearance = 0;
    if (r.nearClearance > 1e8f) r.nearClearance = r.minClearance;
    for (size_t i = 2; i + 1 < wp.size(); i += 2) r.length += std::hypot(wp[i] - wp[i - 2], wp[i + 1] - wp[i - 1]);
    if (r.ok) {
        // Direction to the first waypoint at least 0.5 m away vs heading.
        float tx = wp[2] - ux, ty = wp[3] - uy;
        for (size_t i = 2; i + 1 < wp.size(); i += 2)
            if (std::hypot(wp[i] - ux, wp[i + 1] - uy) > 0.5f) { tx = wp[i] - ux; ty = wp[i + 1] - uy; break; }
        // Relative to the smoothed walking direction, not where the head points.
        r.turnDeg = std::atan2(dirX * ty - dirY * tx, dirX * tx + dirY * ty) * 57.2957795f;
        (void)hx; (void)hy;
    }
    if (r.ok) {
        const float direct = std::hypot(r.goalX - ux, r.goalY - uy);
        if (r.length > detourFactor * direct + detourExtra || std::fabs(r.turnDeg) > maxTurnDeg) {
            r.ok = false;
            r.blocked = true;
        }
    }
    r.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return r;
}
