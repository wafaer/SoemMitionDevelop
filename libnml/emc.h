//
// Created by Administrator on 2025/8/16.
//

#ifndef EMC_H
#define EMC_H

extern int emcTrajInit();
extern int emcMotionInit();
extern int emcTrajSetVelocity(double vel, double ini_maxvel);
extern int emcTrajSetMaxVelocity(double vel);
extern int emcTrajSetAcceleration(double acc);
extern int emcTrajSetMaxAcceleration(double acc);
extern int emcSetupArcBlends(int arcBlendEnable,
        int arcBlendFallbackEnable,
        int arcBlendOptDepth,
        int arcBlendGapCycles,
        double arcBlendRampFreq,
        double arcBlendTangentKinkRatio);
extern int emcAxisSetMinPositionLimit(int axis, double limit);
extern int emcAxisSetMaxPositionLimit(int axis, double limit);
extern int emcAxisSetMaxVelocity(int axis, double vel);
extern int emcAxisSetMaxAcceleration(int axis, double acc);

#endif //EMC_H
