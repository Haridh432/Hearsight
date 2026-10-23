// Host test: ray-cast synthetic depth scenes for the safety layer.
#include "safety.h"
#include "gravity_calib.h"
#include <cmath>
#include <cstdlib>
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

// Gravity "down" in depth-camera axes for a head pitched down by PITCH
// (camera y down, z forward): world down = (0, cos p, sin p).
static const int NRUNS = 20;
static void cameraDown(float g[3], float noiseDeg, std::mt19937& rng) {
    std::normal_distribution<float> n(0, noiseDeg * 3.14159265f / 180);
    const float p = PITCH + n(rng), r = n(rng);
    g[0] = std::sin(r); g[1] = std::cos(p) * std::cos(r); g[2] = std::sin(p) * std::cos(r);
}

static void rotMul(const float R[9], const float v[3], float o[3]) {
    for (int i = 0; i < 3; ++i) o[i] = R[i*3]*v[0] + R[i*3+1]*v[1] + R[i*3+2]*v[2];
}

int main(){
    std::mt19937 rng(5); DepthIntrinsics in{FX,FY,PPX,PPY,W,H};
    int fail = 0;

    // 1. Walking toward a 17 cm step down with head nods, without and with
    //    gravity (1 deg IMU noise). With gravity every run should say drop-off.
    for (int useG = 0; useG < 2; ++useG) {
        int dropStops = 0, stops = 0; float minStop = 1e9f;
        for (int run=0; run<NRUNS; ++run) {
            SafetyLayer L; float firstStop=-1; SafetyReason why = SafetyReason::None;
            for (int f=0; f<=150; ++f) { double t=f/30.0; float remaining=6.0f-1.0f*t;
                PITCH=(27.0f+7.0f*std::sin(2*3.14159*1.0*t+run))*3.14159265f/180; CAM_H=1.6f+0.03f*std::sin(2*3.14159*2.0*t);
                Scene m{"s"}; m.stepZ=remaining; m.stepDrop=0.17f;
                auto img=render(m,rng);
                if (useG) { float g[3]; cameraDown(g, 1.0f, rng); L.setGravity(g[0], g[1], g[2]); }
                auto r=L.process(img.data(),in,0.001f,t);
                if (getenv("DBG2") && run==atoi(getenv("DBG2")) && useG && remaining<2.6f)
                    printf("     rem %.2f st %d rsn %d drop %.2f camH %.2f pitch %.1f floor %d turn %.0f\n", remaining, (int)r.state, (int)r.reason, r.dropDist, r.cameraHeight, r.cameraPitchDeg, r.floorFound, r.turnRateDegS);
                if (r.state==SafetyState::Stop) { firstStop=remaining; why=r.reason;
                    if (getenv("DBG")) printf("   run %d %s: STOP %d at step %.2f | camH %.2f pitch %.1f obs %.2f head %.2f drop %.2f unk %.2f\n",
                        run, useG?"G":"-", (int)r.reason, remaining, r.cameraHeight, r.cameraPitchDeg, r.obstacleDist, r.headDist, r.dropDist, r.unknownDist);
                    break; }
            }
            if (firstStop > 0) { ++stops; minStop = std::min(minStop, firstStop); if (why==SafetyReason::DropOff) ++dropStops; }
        }
        int late = 0; (void)late;
        printf("1 step down, walking + nods, %-10s: %d/%d STOP, %d/%d as drop-off, latest STOP at %.2f m\n",
               useG ? "gravity" : "no gravity", stops, NRUNS, dropStops, NRUNS, minStop);
        if (useG && (stops < NRUNS || dropStops < NRUNS * 9 / 10 || minStop < 1.15f)) ++fail;
    }

    // 1c. Same walk with a constant 1.5 deg gravity bias (IMU / mount error)
    //     plus 1 deg noise: the floor must still be found and the step caught.
    {
        int stops = 0, dropStops = 0, floorFrames = 0, frames = 0; float minStop = 1e9f;
        for (int run=0; run<NRUNS; ++run) {
            SafetyLayer L; float firstStop=-1; SafetyReason why = SafetyReason::None;
            for (int f=0; f<=150; ++f) { double t=f/30.0; float remaining=6.0f-1.0f*t;
                PITCH=(27.0f+7.0f*std::sin(2*3.14159*1.0*t+run))*3.14159265f/180; CAM_H=1.6f+0.03f*std::sin(2*3.14159*2.0*t);
                Scene m{"s"}; m.stepZ=remaining; m.stepDrop=0.17f;
                auto img=render(m,rng);
                const float keep = PITCH; PITCH += 1.5f*3.14159265f/180;
                float g[3]; cameraDown(g, 1.0f, rng); PITCH = keep; L.setGravity(g[0], g[1], g[2]);
                auto r=L.process(img.data(),in,0.001f,t);
                ++frames; if (r.floorFound) ++floorFrames;
                if (r.state==SafetyState::Stop) { firstStop=remaining; why=r.reason; break; }
            }
            if (firstStop > 0) { ++stops; minStop = std::min(minStop, firstStop); if (why==SafetyReason::DropOff) ++dropStops; }
        }
        printf("1c step down, gravity biased 1.5 deg  : %d/%d STOP, %d/%d as drop-off, latest STOP at %.2f m, floor found %.0f%%\n",
               stops, NRUNS, dropStops, NRUNS, minStop, 100.0 * floorFrames / frames);
        if (stops < NRUNS || minStop < 1.15f || floorFrames < frames * 95 / 100) ++fail;
    }
    // 2. Flat floor, sharp nods 25-55 deg in 0.3 s, with gravity: no alarms.
    {
        SafetyLayer L; int alarm=0; Scene s{"flat"}; CAM_H=1.6f;
        for (int f=0; f<300; ++f) { double t=f/30.0; double ph=std::fmod(t,1.5);
            float p = ph<0.3 ? 25+30*(float)(ph/0.3) : ph<0.6 ? 55-30*(float)((ph-0.3)/0.3) : 25; PITCH=p*3.14159265f/180;
            auto img=render(s,rng); float g[3]; cameraDown(g, 1.0f, rng); L.setGravity(g[0],g[1],g[2]);
            auto r=L.process(img.data(),in,0.001f,t);
            if (f>=60 && r.state!=SafetyState::Clear) ++alarm; }
        printf("2 sharp nods on flat floor, gravity   : alarm frames %d  %s\n", alarm, alarm==0?"OK":"FAIL");
        if (alarm) ++fail;
    }

    // 3. Calibration: phone rigid with the camera (unknown mount rotation),
    //    head poses varying; floor normal measured with 1 deg noise.
    {
        const float a = 0.9f, b = -0.4f, c = 2.2f;   // arbitrary mount (rad)
        float Rz[9]={std::cos(c),-std::sin(c),0, std::sin(c),std::cos(c),0, 0,0,1};
        float Rx[9]={1,0,0, 0,std::cos(a),-std::sin(a), 0,std::sin(a),std::cos(a)};
        float Ry[9]={std::cos(b),0,std::sin(b), 0,1,0, -std::sin(b),0,std::cos(b)};
        float T[9], M[9];   // M = Rz*Rx*Ry : camera <- phone
        for (int i=0;i<3;i++) for (int j=0;j<3;j++){ T[i*3+j]=0; for(int k=0;k<3;k++) T[i*3+j]+=Rx[i*3+k]*Ry[k*3+j]; }
        for (int i=0;i<3;i++) for (int j=0;j<3;j++){ M[i*3+j]=0; for(int k=0;k<3;k++) M[i*3+j]+=Rz[i*3+k]*T[k*3+j]; }
        float Mt[9]; for (int i=0;i<3;i++) for (int j=0;j<3;j++) Mt[i*3+j]=M[j*3+i];
        for (int rigid = 1; rigid >= 0; --rigid) {
            GravityCalib C; std::uniform_real_distribution<float> U(-1,1);
            for (int f=0; f<1800; ++f) { double t=f/30.0;
                PITCH=(25.0f+12.0f*std::sin(2*3.14159*0.3*t))*3.14159265f/180;
                float camUp[3], d[3]; cameraDown(d, 1.0f, rng); for(int i=0;i<3;i++) camUp[i]=-d[i];
                float trueCam[3]={0,-std::cos(PITCH),-std::sin(PITCH)};
                float phoneUp[3];
                if (rigid) rotMul(Mt, trueCam, phoneUp);
                else { phoneUp[0]=U(rng); phoneUp[1]=U(rng); phoneUp[2]=1; float l=std::sqrt(phoneUp[0]*phoneUp[0]+phoneUp[1]*phoneUp[1]+1); for(float& v:phoneUp) v/=l; }
                C.add(phoneUp, camUp, t);
            }
            float err = -1;
            if (C.ok()) { float probe[3]={0,0,1}, ph[3], back[3]; rotMul(Mt, probe, ph); C.toCamera(ph, back);
                err = std::acos(std::min(1.0f, back[2])) * 57.3f; }
            const bool ok = rigid ? (C.ok() && err < 2.0f) : !C.ok();
            printf("3 calibration, phone %-10s : ok %d, spread %.1f deg, residual %.2f deg, axis error %.2f deg  %s\n",
                   rigid ? "rigid" : "not rigid", C.ok(), C.spreadDeg(), C.residualDeg(), err, ok?"OK":"FAIL");
            if (!ok) ++fail;
        }
    }
    printf("gravity failures: %d\n", fail);
    return fail;
}
