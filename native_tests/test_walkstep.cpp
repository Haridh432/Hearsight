// Host test: ray-cast synthetic depth scenes for the safety layer.
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
 case SafetyReason::DropOff:return "drop-off";case SafetyReason::UnknownFloor:return "unknown-floor";case SafetyReason::NoFloor:return "no-floor";default:return "-";}}
int main(){
    std::mt19937 rng(5); DepthIntrinsics in{FX,FY,PPX,PPY,W,H};
    for (int run=0; run<5; ++run) {
        SafetyLayer L; float firstCaution=-1, firstStop=-1;
        for (int f=0; f<=150; ++f) { double t=f/30.0; float remaining=6.0f-1.0f*t;      // step 6 m ahead, walking 1 m/s
            PITCH=(27.0f+7.0f*std::sin(2*3.14159*1.0*t+run))*3.14159265f/180; CAM_H=1.6f+0.03f*std::sin(2*3.14159*2.0*t);
            Scene m{"s"}; m.stepZ=remaining; m.stepDrop=0.17f;
            auto img=render(m,rng); auto r=L.process(img.data(),in,0.001f,t);
            if (r.state==SafetyState::Caution && firstCaution<0 && r.reason==SafetyReason::DropOff) firstCaution=remaining;
            if (r.state==SafetyState::Stop && firstStop<0) { firstStop=remaining; printf("run %d: STOP (%s) when step was %.2f m ahead; first drop CAUTION at %.2f m\n", run, rs(r.reason), remaining, firstCaution); break; }
        }
        if (firstStop<0) printf("run %d: NO STOP (first caution %.2f)\n", run, firstCaution);
    }
}
