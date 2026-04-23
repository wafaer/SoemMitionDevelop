//
// Created by Administrator on 2025/8/16.
//

#ifndef EMCGLB_H
#define EMCGLB_H

typedef struct AxisConfig_t {
    int Inited;
    unsigned char Type;
    double MaxVel;
    double MaxAccel;
    double MinLimit;
    double MaxLimit;
} AxisConfig_t;

typedef struct TrajConfig_t {
    int Inited;
    int Axis;
    double MaxAccel;
    double MaxVel;
    int AxisMask;
    double LinearUnits;
    double AngularUnits;
    int MotionId;
} TrajConfig_t;

#endif //EMCGLB_H
