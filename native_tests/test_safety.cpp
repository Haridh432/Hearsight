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
const float CAM_H=1.6f, PITCH=27.0f*3.14159265f/180;   // head height, tilted down
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
    std::vector<Scene> sc;
    sc.push_back({"empty corridor floor"});
    sc.push_back({"box 0.5 m tall at 1.0 m",{{-0.25f,0,1.0f,0.25f,0.5f,1.4f}}});
    sc.push_back({"box 0.5 m tall at 2.0 m",{{-0.25f,0,2.0f,0.25f,0.5f,2.4f}}});
    sc.push_back({"chair seat 0.45 m at 1.6 m",{{-0.25f,0.4f,1.6f,0.25f,0.47f,2.0f}}});
    sc.push_back({"head-height board 1.45-1.9 m at 1.0 m",{{-0.5f,1.45f,1.0f,0.5f,1.9f,1.05f}}});
    {Scene s{"step down 17 cm at 1.3 m"}; s.stepZ=1.3f; s.stepDrop=0.17f; sc.push_back(s);}
    {Scene s{"hole / no depth 0.8-1.1 m"}; s.holeZ0=0.8f; s.holeZ1=1.1f; sc.push_back(s);}
    {Scene s{"wall at 0.9 m"}; s.wallZ=0.9f; sc.push_back(s);}
    sc.push_back({"box beside path (x 0.8 m) at 1.0 m",{{0.7f,0,1.0f,1.1f,0.6f,1.4f}}});
    sc.push_back({"door lintel 2.0-2.2 m over open doorway, 1.0 m",{{-0.5f,2.0f,1.0f,0.5f,2.2f,1.1f}}});
    std::mt19937 rng(7);
    DepthIntrinsics in{FX,FY,PPX,PPY,W,H};
    // Every scene is approached on foot: objects start 2 m further away and
    // move closer over 2 s (user walking at 1 m/s), ending at their position.
    for (auto& s: sc){ SafetyLayer L; SafetyResult r; for(int f=0;f<=60;f++){ float off=2.0f*(60-f)/60.0f; Scene m=s;
            for(auto& b: m.boxes){ b.z0+=off; b.z1+=off; } if(m.stepZ>0) m.stepZ+=off; if(m.holeZ0>0){ m.holeZ0+=off; m.holeZ1+=off; } if(m.wallZ>0) m.wallZ+=off;
            auto img=render(m,rng); r=L.process(img.data(),in,0.001f,f/30.0); }
        printf("%-40s -> %-7s %-13s dist %.2f | floor %d camH %.2f pitch %.1f | obs %.2f head %.2f drop %.2f unk %.2f | %d pts %.1f ms\n",
            s.name.c_str(), st(r.state), rs(r.reason), r.distance, r.floorFound, r.cameraHeight, r.cameraPitchDeg,
            r.obstacleDist, r.headDist, r.dropDist, r.unknownDist, r.points, r.ms); }
}
