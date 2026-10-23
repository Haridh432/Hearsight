// Host test: fixes from the head-mounted walk test (2026-10-16).
#include "safety.h"
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>
struct Box { float x0,y0,z0,x1,y1,z1; };            // world: x right, y up, z forward (m)
struct Scene { std::string name; std::vector<Box> boxes; float stepZ=-1, stepDrop=0; float holeZ0=-1, holeZ1=-1; float wallZ=-1; };
const int W=640, H=480; const float FX=390, FY=390, PPX=320, PPY=240;
float CAM_H=1.6f, PITCH=27.0f*3.14159265f/180;   // head height, tilted down
float rayBox(const float o[3], const float d[3], const Box& b) {
    float tmin=0, tmax=1e9; const float lo[3]={b.x0,b.y0,b.z0}, hi[3]={b.x1,b.y1,b.z1};
    for (int i=0;i<3;i++){ if (std::fabs(d[i])<1e-9){ if(o[i]<lo[i]||o[i]>hi[i]) return -1; continue; }
        float t1=(lo[i]-o[i])/d[i], t2=(hi[i]-o[i])/d[i]; if(t1>t2) std::swap(t1,t2);
        tmin=std::max(tmin,t1); tmax=std::min(tmax,t2); if(tmin>tmax) return -1; }
    return tmin; }
std::vector<uint16_t> render(const Scene& s, std::mt19937& rng) {
    std::vector<uint16_t> img(W*H, 0); std::normal_distribution<float> nz(0,1);
    const float o[3]={0,CAM_H,0}; const float c=std::cos(PITCH), sn=std::sin(PITCH);
    for (int v=0; v<H; v++) for (int u=0; u<W; u++) {
        float xc=(u-PPX)/FX, yc=(v-PPY)/FY;                   // camera ray (y down), z=1
        float d[3]={xc, -yc*c - sn, -yc*(-sn) + c};          // rotate camera -> world (pitch down)
        // camera forward (0,0,1) -> world (0,-sn,c); camera down (0,1,0) -> world (0,-c,-sn)
        d[1] = -yc*c - 1*sn; d[2] = -yc*sn*-1*0 + c*1 - yc*sn;
        float t=1e9;
        // floor (with optional step down beyond stepZ)
        if (d[1] < 0) { float tf=(0-o[1])/d[1]; float zf=o[2]+d[2]*tf;
            if (s.stepZ>0 && zf>s.stepZ) { tf=(-s.stepDrop-o[1])/d[1]; zf=o[2]+d[2]*tf;
                float tEdge=(s.stepZ-o[2])/d[2]; float yEdge=o[1]+d[1]*tEdge; if (yEdge<0 && yEdge>-s.stepDrop) tf=tEdge; }
            if (!(s.holeZ0>0 && zf>s.holeZ0 && zf<s.holeZ1)) t=std::min(t,tf); else t=std::min(t,1e8f); }
        if (s.wallZ>0 && d[2]>0) t=std::min(t,(s.wallZ-o[2])/d[2]);
        for (const Box& b: s.boxes){ float tb=rayBox(o,d,b); if(tb>0) t=std::min(t,tb); }
        if (t>=1e8f) continue;
        float zcam = t * 1.0f;                                  // depth along camera z (ray z=1)
        if (zcam>6) continue;
        zcam += 0.001f*zcam*zcam*nz(rng);
        img[v*W+u]=static_cast<uint16_t>(zcam/0.001f);
    }
    return img; }
const char* st(SafetyState s){return s==SafetyState::Stop?"STOP":s==SafetyState::Caution?"CAUTION":"clear";}
const char* rs(SafetyReason r){switch(r){case SafetyReason::Obstacle:return "obstacle";case SafetyReason::HeadHeight:return "head";
 case SafetyReason::DropOff:return "drop-off";case SafetyReason::UnknownFloor:return "unknown-floor";case SafetyReason::NoFloor:return "no-floor";case SafetyReason::Stairs:return "stairs";default:return "-";}}

const float DEG=3.14159265f/180;
int main(){
    std::mt19937 rng(5); DepthIntrinsics in{FX,FY,PPX,PPY,W,H};
    int fail=0;

    // 1. Fast head nods over flat floor: pitch swings 15-45 deg at 0.7 Hz
    //    (seen on the phone: 42 -> 15 -> 37 deg within ~2 s). No drop-off.
    {
        SafetyLayer L; int drop=0, alarm=0; Scene s{"flat"};
        for (int f=0; f<300; ++f) { double t=f/30.0;
            PITCH=(30.0f+15.0f*std::sin(2*3.14159f*0.7f*t))*DEG; CAM_H=1.6f+0.03f*std::sin(2*3.14159f*2*t);
            auto img=render(s,rng); auto r=L.process(img.data(),in,0.001f,t);
            if (f>=60) { if (r.reason==SafetyReason::DropOff) ++drop; if (r.state!=SafetyState::Clear) ++alarm; } }
        printf("1 fast nods 15-45 deg, flat floor     : drop-off frames %d, any alarm %d  %s\n", drop, alarm, drop==0?"OK":"FAIL");
        if (drop) ++fail;
    }
    // 2. Standing 1.0 m from a box (STOP), then the head comes up from
    //    40 to 5 deg in 0.3 s: the box leaves the view. STOP must be held
    //    while it is out of view, then released after the hold.
    {
        SafetyLayer L; Scene s{"box",{{-0.25f,0,1.0f,0.25f,0.30f,1.4f}}}; CAM_H=1.6f;
        int held=0; double lastStop=-1; SafetyResult r;
        for (int f=0; f<210; ++f) { double t=f/30.0;
            float p = t<3.0 ? 40.0f : t<3.3 ? 40.0f-35.0f*(float)((t-3.0)/0.3) : 5.0f; PITCH=p*DEG;
            auto img=render(s,rng); r=L.process(img.data(),in,0.001f,t);
            if (r.state==SafetyState::Stop) lastStop=t;
            if (t>=3.3 && r.state==SafetyState::Stop) ++held; }
        const double heldSec = lastStop-3.3;
        printf("2 look up near a box (out of view)    : STOP held %.1f s after the box left the view  %s\n",
               heldSec, heldSec>=1.5 && heldSec<=3.0 ? "OK":"FAIL");
        if (!(heldSec>=1.5 && heldSec<=3.0)) ++fail;
    }
    // 2b. Same, but the user stepped back to 2.2 m (CAUTION) before
    //     raising the head: the CAUTION must be held too.
    {
        SafetyLayer L; Scene s{"box",{{-0.25f,0,2.2f,0.25f,0.30f,2.6f}}}; CAM_H=1.6f;
        double lastAlarm=-1;
        for (int f=0; f<210; ++f) { double t=f/30.0;
            float p = t<3.0 ? 40.0f : t<3.3 ? 40.0f-45.0f*(float)((t-3.0)/0.3) : -5.0f; PITCH=p*DEG;
            auto img=render(s,rng); auto r=L.process(img.data(),in,0.001f,t);
            if (r.state!=SafetyState::Clear) lastAlarm=t; }
        const double heldSec = lastAlarm-3.3;
        printf("2b look up 2.2 m from a box (CAUTION)  : alert held %.1f s after it left the view  %s\n",
               heldSec, heldSec>=1.5 && heldSec<=3.0 ? "OK":"FAIL");
        if (!(heldSec>=1.5 && heldSec<=3.0)) ++fail;
    }
    // 1b. Very fast nod down (38 -> 51 deg in 0.3 s, seen on the phone)
    //     over flat floor, repeated: no drop-off.
    {
        SafetyLayer L; int drop=0; Scene s{"flat"}; CAM_H=1.6f;
        for (int f=0; f<300; ++f) { double t=f/30.0; double ph=std::fmod(t,1.5);
            float p = ph<0.3 ? 25+30*(float)(ph/0.3) : ph<0.6 ? 55-30*(float)((ph-0.3)/0.3) : 25; PITCH=p*DEG;
            auto img=render(s,rng); auto r=L.process(img.data(),in,0.001f,t);
            if (f>=60 && r.reason==SafetyReason::DropOff) ++drop; }
        printf("1b sharp nods 25-55 deg in 0.3 s       : drop-off frames %d  %s\n", drop, drop==0?"OK":"FAIL");
        if (drop) ++fail;
    }
    // 3. Walking speed: first STOP distance to a box when approaching at
    //    0.5 m/s and 1.5 m/s from 4 m. Faster walking must stop earlier.
    float stopAt[2];
    for (int k=0; k<2; ++k) {
        const float v = k==0 ? 0.5f : 1.5f;
        SafetyLayer L; PITCH=27*DEG; CAM_H=1.6f; stopAt[k]=-1;
        for (int f=0; f<400; ++f) { double t=f/30.0; float z=4.0f-v*(float)t; if (z<0.4f) break;
            Scene s{"box",{{-0.25f,0,z,0.25f,0.5f,z+0.4f}}};
            auto img=render(s,rng); auto r=L.process(img.data(),in,0.001f,t);
            if (r.state==SafetyState::Stop) { stopAt[k]=z; break; } }
        printf("3 approach a box at %.1f m/s           : first STOP at %.2f m\n", v, stopAt[k]);
    }
    // Same 1.5 m/s approach, but the box is missed on every other frame
    // (seen on the phone: detection at the point-count threshold).
    float stopFlicker=-1;
    {
        SafetyLayer L; PITCH=27*DEG; CAM_H=1.6f;
        for (int f=0; f<400; ++f) { double t=f/30.0; float z=4.0f-1.5f*(float)t; if (z<0.4f) break;
            Scene s{"box"}; if (f%2==0) s.boxes.push_back({-0.25f,0,z,0.25f,0.5f,z+0.4f});
            auto img=render(s,rng); auto r=L.process(img.data(),in,0.001f,t);
            if (r.state==SafetyState::Stop) { stopFlicker=z; break; } }
        printf("3 1.5 m/s, box seen every other frame : first STOP at %.2f m\n", stopFlicker);
    }
    const bool speedOk = stopAt[0]>=1.0f && stopAt[0]<=1.35f && stopAt[1]>=1.5f && stopFlicker>=1.45f;
    printf("  speed-dependent STOP                : %s\n", speedOk?"OK":"FAIL");
    if (!speedOk) ++fail;
    printf("walk-fix failures: %d\n", fail);
    return fail;
}
