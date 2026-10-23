// Host test: trip hazards (3-10 cm objects) with realistic depth noise.
#define main test_map_main
#include "test_map.cpp"
#undef main

struct TripRun { int tripFrames, otherAlarms; float firstDist; };

static TripRun run(const Scene& sc, float noise, bool nod, int frames = 90) {
    g_noise = noise;
    std::mt19937 rng(9);
    SafetyLayer L;
    DepthIntrinsics in{FX, FY, CX, CY, W, H};
    std::vector<uint16_t> depth; std::vector<uint8_t> ids;
    TripRun r{0, 0, -1};
    for (int f = 0; f < frames; ++f) {
        const double t = f / 30.0;
        const float pitch = nod ? 30 + 12 * std::sin(2 * PI * 0.8f * (float)t) : 30;
        render(makeCam(0, 1.6f, 0, 0, pitch), sc, depth, ids, rng);
        SafetyResult s = L.process(depth.data(), in, 0.001f, t);
        if (f < 30) continue;   // eye height learning
        if (s.reason == SafetyReason::TripHazard) { ++r.tripFrames; if (r.firstDist < 0) r.firstDist = s.distance; }
        else if (s.state != SafetyState::Clear) ++r.otherAlarms;
    }
    return r;
}

int main() {
    int fail = 0;
    auto box = [](float h, float z0, float z1) { Scene s; s.boxes.push_back({-0.3f, 0, z0, 0.3f, h, z1, 1}); return s; };
    struct Case { const char* name; Scene sc; float noise; bool nod; bool expectTrip; };
    const Case cases[] = {
        {"5 cm cable bundle at 1.5 m", box(0.05f, 1.5f, 1.62f), 0.003f, false, true},
        {"8 cm door threshold at 1.0 m", box(0.08f, 1.0f, 1.1f), 0.003f, false, true},
        {"5 cm cable at 1.5 m, head nods", box(0.05f, 1.5f, 1.62f), 0.003f, true, true},
        {"2 cm rug edge at 1.2 m (no alert)", box(0.02f, 1.2f, 2.0f), 0.003f, false, false},
        {"flat floor, head nods (no alert)", Scene{}, 0.003f, true, false},
        {"flat floor, 2x noise (no alert)", Scene{}, 0.006f, false, false},
    };
    for (const Case& c : cases) {
        const TripRun r = run(c.sc, c.noise, c.nod);
        const bool ok = c.expectTrip ? (r.tripFrames >= 30 && r.otherAlarms == 0) : (r.tripFrames == 0 && r.otherAlarms == 0);
        printf("%-38s: trip frames %2d/60, first at %.2f m, other alarms %d  %s\n", c.name, r.tripFrames, r.firstDist,
               r.otherAlarms, ok ? "OK" : "FAIL");
        if (!ok) ++fail;
    }
    printf("trip failures: %d\n", fail);
    return fail;
}
