// Sensor health: blocked lens, frozen frames, lag, frame gaps, recovery.
#include <cstdio>

#include "sensor_health.h"

static int fails = 0;
static void check(const char* name, bool ok) { printf("  %-44s: %s\n", name, ok ? "OK" : "FAIL"); if (!ok) ++fails; }

// Runs frames at 30 Hz from t0 to t1 (s) with the given input template.
static SensorHealthResult run(SensorHealth& h, double t0, double t1, SensorHealthInput in, bool frozen = false) {
    SensorHealthResult r;
    static double frameMs = 0;
    for (double t = t0; t < t1; t += 1.0 / 30) {
        in.tSec = t;
        if (!frozen) frameMs += 33.3;
        in.frameTimeMs = frameMs;
        r = h.update(in);
    }
    return r;
}

int main() {
    printf("sensor health\n");
    SensorHealthInput ok; ok.depthValid = 0.6f; ok.latencyMs = 40; ok.inferenceMs = 12;
    SensorHealth h;
    check("normal frames -> ok", run(h, 0, 2, ok).level == 0);
    SensorHealthInput cov = ok; cov.depthValid = 0.02f;
    check("covered 0.3 s -> still ok", run(h, 2, 2.3, cov).level == 0);
    SensorHealthResult r = run(h, 2.3, 2.7, cov);
    check("covered 0.7 s -> fault blocked", r.level == 2 && r.reason == SensorFault::Blocked);
    check("uncovered 0.5 s -> fault holds", run(h, 2.7, 3.2, ok).level == 2);
    check("uncovered 1.2 s -> ok", run(h, 3.2, 4.0, ok).level == 0);
    r = run(h, 4.0, 4.7, ok, true);
    check("same frame 0.7 s -> fault frozen", r.level == 2 && r.reason == SensorFault::Frozen);
    run(h, 4.7, 6.0, ok);
    SensorHealthInput lag = ok; lag.latencyMs = 600;
    r = run(h, 6.0, 7.5, lag);
    check("lag 600 ms -> fault lag", r.level == 2 && r.reason == SensorFault::Lag);
    SensorHealth h2;
    run(h2, 0, 1, ok);
    run(h2, 1.5, 1.6, ok);   // 0.5 s gap
    r = h2.current();
    check("0.5 s frame gap -> degraded", r.level == 1 && r.reason == SensorFault::FrameGap);
    check("1 s later -> ok", run(h2, 1.6, 2.8, ok).level == 0);
    SensorHealthInput slow = ok; slow.inferenceMs = 600;
    check("slow model -> degraded", run(h2, 2.8, 3.0, slow).level == 1);
    SensorHealthInput noclk = ok; noclk.latencyMs = -1;
    SensorHealth h3;
    check("unknown clock -> no lag fault", run(h3, 0, 2, noclk).level == 0);
    // Startup: lag readings in the first 3 s are ignored.
    SensorHealth h4; SensorHealthInput big = ok; big.latencyMs = 1140;
    check("lag 1140 ms in the first 2 s -> ok", run(h4, 0, 2, big).level == 0);
    check("lag 1140 ms after 3 s -> fault", run(h4, 2, 4.5, big).level == 2);
    // Start-up warm-up: degraded ("Wait") for warmupSec, longer while depth is sparse.
    SensorHealth h5;
    SensorHealthResult w = run(h5, 0, 0.5, ok);
    check("start-up 0.5 s -> degraded (warmup)", w.level == 1 && w.reason == SensorFault::Warmup);
    check("start-up 1.2 s, good depth -> ok", run(h5, 0.5, 1.2, ok).level == 0);
    SensorHealth h6; SensorHealthInput sparse = ok; sparse.depthValid = 0.15f;
    check("start-up 1.5 s, sparse depth -> still warmup", run(h6, 0, 1.5, sparse).reason == SensorFault::Warmup);
    check("start-up 3.2 s, sparse depth -> ok (warm-up capped)", run(h6, 1.5, 3.2, sparse).level == 0);
    printf("health failures: %d\n", fails);
    return fails ? 1 : 0;
}
