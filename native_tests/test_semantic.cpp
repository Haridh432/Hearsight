// Host test: safety layer with scene-segmentation roles (synthetic scenes).
// Each scene is rendered twice: depth + a per-pixel role image (what the
// segmentation model would label), and run with and without semantics.
#include "safety.h"
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>
struct Box { float x0,y0,z0,x1,y1,z1; uint8_t role; };
struct Scene { std::string name; std::vector<Box> boxes; float stepZ=-1, stepDrop=0; bool stepIsStairs=false;
               float labelNoise=0;      // fraction of cells given a random wrong label
               float reflZ=-1, ceilH=3.0f; };   // glossy floor from reflZ: mirror of a ceiling at ceilH
const int W=640, H=480, GW=96, GH=72; const float FX=390, FY=390, PPX=320, PPY=240;
const float CAM_H=1.6f, PITCH=27.0f*3.14159265f/180;
float rayBox(const float o[3], const float d[3], const Box& b) {
    float tmin=0, tmax=1e9; const float lo[3]={b.x0,b.y0,b.z0}, hi[3]={b.x1,b.y1,b.z1};
    for (int i=0;i<3;i++){ if (std::fabs(d[i])<1e-9){ if(o[i]<lo[i]||o[i]>hi[i]) return -1; continue; }
        float t1=(lo[i]-o[i])/d[i], t2=(hi[i]-o[i])/d[i]; if(t1>t2) std::swap(t1,t2);
        tmin=std::max(tmin,t1); tmax=std::min(tmax,t2); if(tmin>tmax) return -1; }
    return tmin; }
void render(const Scene& s, std::mt19937& rng, std::vector<uint16_t>& img, std::vector<uint8_t>& grid) {
    img.assign(W*H, 0); std::vector<uint8_t> roles(W*H, kSemOther); std::normal_distribution<float> nz(0,1);
    const float o[3]={0,CAM_H,0}; const float c=std::cos(PITCH), sn=std::sin(PITCH);
    for (int v=0; v<H; v++) for (int u=0; u<W; u++) {
        float xc=(u-PPX)/FX, yc=(v-PPY)/FY;
        float d[3]={xc, -yc*c - sn, c - yc*sn};
        float t=1e9; uint8_t role=kSemOther;
        if (d[1] < 0) { float tf=(0-o[1])/d[1]; float zf=o[2]+d[2]*tf; role=kSemWalkable;
            // Glossy floor: the camera sees the ceiling's mirror image below the floor.
            if (s.reflZ>0 && zf>s.reflZ) { tf=(-s.ceilH-o[1])/d[1]; role=kSemOther; }   // looks like the wall it mirrors
            if (s.stepZ>0 && zf>s.stepZ) { tf=(-s.stepDrop-o[1])/d[1];
                float tEdge=(s.stepZ-o[2])/d[2]; float yEdge=o[1]+d[1]*tEdge; if (yEdge<0 && yEdge>-s.stepDrop) tf=tEdge;
                if (s.stepIsStairs) role=kSemStairs; }
            t=tf; }
        for (const Box& b: s.boxes){ float tb=rayBox(o,d,b); if(tb>0 && tb<t){ t=tb; role=b.role; } }
        if (t>6) continue;
        img[v*W+u]=static_cast<uint16_t>((t + 0.001f*t*t*nz(rng))/0.001f);
        roles[v*W+u]=role;
    }
    grid.assign(GW*GH, kSemOther);
    std::uniform_real_distribution<float> U(0,1); std::uniform_int_distribution<int> R(0,3);
    for (int gy=0; gy<GH; gy++) for (int gx=0; gx<GW; gx++) {
        uint8_t r = roles[(gy*H/GH + H/GH/2)*W + gx*W/GW + W/GW/2];
        if (U(rng) < s.labelNoise) r = static_cast<uint8_t>(R(rng));
        grid[gy*GW+gx] = r;
    }
}
const char* st(SafetyState s){return s==SafetyState::Stop?"STOP":s==SafetyState::Caution?"CAUTION":"clear";}
const char* rs(SafetyReason r){switch(r){case SafetyReason::Obstacle:return "obstacle";case SafetyReason::HeadHeight:return "head";
 case SafetyReason::DropOff:return "drop-off";case SafetyReason::UnknownFloor:return "unknown";case SafetyReason::NoFloor:return "no-floor";
 case SafetyReason::Stairs:return "stairs";default:return "-";}}
int main(){
    std::vector<Scene> sc;
    sc.push_back({"empty corridor"});
    {Scene s{"empty corridor, 10% label noise"}; s.labelNoise=0.10f; sc.push_back(s);}
    sc.push_back({"low 6 cm box at 1.0 m (obstacle label)",{{-0.25f,0,1.0f,0.25f,0.06f,1.4f,kSemObstacle}}});
    sc.push_back({"low 6 cm mat at 1.0 m (floor label)",{{-0.25f,0,1.0f,0.25f,0.06f,1.4f,kSemWalkable}}});
    {Scene s{"step down 17 cm at 1.3 m, stairs label"}; s.stepZ=1.3f; s.stepDrop=0.17f; s.stepIsStairs=true; sc.push_back(s);}
    {Scene s{"step down 17 cm at 1.3 m, floor label"}; s.stepZ=1.3f; s.stepDrop=0.17f; sc.push_back(s);}
    sc.push_back({"stairs up: 3 steps from 2.4 m",{{-0.6f,0,2.4f,0.6f,0.17f,4,kSemStairs},{-0.6f,0,2.7f,0.6f,0.34f,4,kSemStairs},
                                                   {-0.6f,0,3.0f,0.6f,0.51f,4,kSemStairs}}});
    sc.push_back({"box 0.5 m at 2.0 m",{{-0.25f,0,2.0f,0.25f,0.5f,2.4f,kSemObstacle}}});
    // Phone (polished hall floor): ceiling reflections read as points ~3 m
    // below the floor -> constant false drop-offs and "No clear way".
    {Scene s{"glossy floor from 1.5 m (reflection of a 1 m desk)"}; s.reflZ=1.5f; s.ceilH=1.0f; sc.push_back(s);}
    std::mt19937 rng(7);
    DepthIntrinsics in{FX,FY,PPX,PPY,W,H};
    int fail = 0;
    for (auto& s: sc){
        for (int useSem=0; useSem<2; useSem++){
            SafetyLayer L; SafetyResult r; std::vector<uint16_t> img; std::vector<uint8_t> grid;
            SemanticInput sem; sem.gridW=GW; sem.gridH=GH; sem.fx=FX; sem.fy=FY; sem.ppx=PPX; sem.ppy=PPY; sem.colorW=W; sem.colorH=H;
            int alarms = 0;
            for(int f=0;f<=60;f++){ float off=2.0f*(60-f)/60.0f; Scene m=s;   // walk in from 2 m further
                for(auto& b: m.boxes){ b.z0+=off; if (b.z1<4) b.z1+=off; } if(m.stepZ>0) m.stepZ+=off;
                render(m,rng,img,grid); sem.roles=grid.data();
                r=L.process(img.data(),in,0.001f,f/30.0, useSem ? &sem : nullptr);
                if (r.state!=SafetyState::Clear) ++alarms; }
            printf("%-42s %-4s -> %-7s %-9s dist %.2f | camH %.2f | stairs %.2f low %.2f | alarm frames %d\n",
                s.name.c_str(), useSem?"SEM":"geo", st(r.state), rs(r.reason), r.distance, r.cameraHeight,
                r.stairsDist, r.lowObstacleDist, alarms);
            if (useSem && s.name.rfind("empty",0)==0 && alarms) ++fail;
            // Reflections must not be a drop-off with segmentation; a real 17 cm step still is.
            if (useSem && s.name.rfind("glossy",0)==0 && (r.reason==SafetyReason::DropOff || r.dropDist>=0)) {
                printf("   FAIL: glossy floor reflection read as a drop-off\n"); ++fail; }
            if (useSem && s.name.rfind("step down 17 cm",0)==0 && r.reason!=SafetyReason::DropOff &&
                r.reason!=SafetyReason::Stairs) { printf("   FAIL: real step not a drop-off / stairs\n"); ++fail; }
        }
    }
    // START while looking at a desk: a 0.75 m tabletop fills the view for
    // the first 2 s, then the user turns to the clear floor. The eye height
    // must come out at 1.60 m (not 0.85 m above the desk) and stay clear.
    {
        SafetyLayer L; SafetyResult r; std::vector<uint16_t> img; std::vector<uint8_t> grid;
        Scene desk{"desk"}; desk.boxes.push_back({-2.0f,0,0.2f,2.0f,0.75f,4.0f,kSemOther});
        Scene clear{"clear"};
        int alarmsAfter = 0;
        for (int f=0; f<180; f++) {
            render(f < 60 ? desk : clear, rng, img, grid);
            r = L.process(img.data(), in, 0.001f, f/30.0);
            if (f >= 150 && r.state != SafetyState::Clear) ++alarmsAfter;
        }
        const bool ok = r.floorFound && std::fabs(r.cameraHeight - CAM_H) < 0.05f && alarmsAfter == 0;
        printf("%-42s geo  -> %-7s camH %.2f | alarm frames in last 1 s %d  %s\n", "START facing a desk, then clear floor",
               st(r.state), r.cameraHeight, alarmsAfter, ok ? "OK" : "FAIL");
        if (!ok) ++fail;
    }
    printf("semantic false-alarm failures: %d\n", fail);
    return fail;
}
