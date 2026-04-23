//
// Created by Administrator on 2025/8/16.
//

#ifndef EMCPOS_H
#define EMCPOS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "posemath.h"

    typedef struct EmcPose {
        PmCartesian tran;
        double a, b, c;
        double u, v, w;
    } EmcPose;

    typedef enum {
        EMCPOSE_ERR_OK = 0,
        EMCPOSE_ERR_FAIL = -1,
        EMCPOSE_ERR_INPUT_MISSING = -2,
        EMCPOSE_ERR_OUTPUT_MISSING = -3,
        EMCPOSE_ERR_ALL
    } EmcPoseErr;

#define ZERO_EMC_POSE(pos) do { \
pos.tran.x = 0.0;               \
pos.tran.y = 0.0;               \
pos.tran.z = 0.0;               \
pos.a = 0.0;                    \
pos.b = 0.0;                    \
pos.c = 0.0;                    \
pos.u = 0.0;                    \
pos.v = 0.0;                    \
pos.w = 0.0; } while(0)

    void emcPoseZero(EmcPose * const pos);

    int emcPoseValid(EmcPose const * const pose);
    int emcPoseSelfAdd(EmcPose * const self, EmcPose const * const p2);
    int emcPoseSub(EmcPose const * const p1, EmcPose const * const p2, EmcPose * const out);
    int emcPoseSelfSub(EmcPose * const self, EmcPose const * const p2);
    int pmCartesianToEmcPose(PmCartesian const * const xyz,
            PmCartesian const * const abc, PmCartesian const * const uvw, EmcPose * const pose);
    int emcPoseToPmCartesian(EmcPose const * const pose,
            PmCartesian * const xyz, PmCartesian * const abc, PmCartesian * const uvw);
    int emcPoseGetXYZ(EmcPose const * const pose, PmCartesian * const xyz);

#ifdef __cplusplus
}
#endif

#endif //EMCPOS_H
