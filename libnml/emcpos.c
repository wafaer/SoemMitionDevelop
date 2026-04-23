//
// Created by Administrator on 2025/8/19.
//

#include "emcpos.h"
#include <math.h>
#include "rtapi/rtapi.h"

int emcPoseValid(EmcPose const * const pose)
{

    if (!pose ||
            isnan(pose->tran.x) ||
            isnan(pose->tran.y) ||
            isnan(pose->tran.z) ||
            isnan(pose->a) ||
            isnan(pose->b) ||
            isnan(pose->c) ||
            isnan(pose->u) ||
            isnan(pose->v) ||
            isnan(pose->w)) {
        return 0;
            } else {
                return 1;
            }
}

int emcPoseAdd(EmcPose const * const p1, EmcPose const * const p2, EmcPose * const out)
{
    if (!p1 || !p2 || !out) return -1;

    pmCartCartAdd(&p1->tran, &p2->tran, &out->tran);
    out->a = p1->a + p2->a;
    out->b = p1->b + p2->b;
    out->c = p1->c + p2->c;
    out->u = p1->u + p2->u;
    out->v = p1->v + p2->v;
    out->w = p1->w + p2->w;

    return EMCPOSE_ERR_OK;
}

int emcPoseSelfAdd(EmcPose * const self, EmcPose const * const p2)
{
    if (!self || !p2) return -1;

    return emcPoseAdd(self, p2, self);
}

void emcPoseZero(EmcPose * const pos) {
#ifdef EMCPOSE_PEDANTIC
    if(!pos) {
        return EMCPOSE_ERR_INPUT_MISSING;
    }
#endif

    pos->tran.x = 0.0;
    pos->tran.y = 0.0;
    pos->tran.z = 0.0;
    pos->a = 0.0;
    pos->b = 0.0;
    pos->c = 0.0;
    pos->u = 0.0;
    pos->v = 0.0;
    pos->w = 0.0;
}

int emcPoseSub(EmcPose const * const p1, EmcPose const * const p2, EmcPose * const out)
{
    if (!p1 || !p2) {
        return EMCPOSE_ERR_INPUT_MISSING;
    }

    pmCartCartSub(&p1->tran, &p2->tran, &out->tran);
    out->a = p1->a - p2->a;
    out->b = p1->b - p2->b;
    out->c = p1->c - p2->c;
    out->u = p1->u - p2->u;
    out->v = p1->v - p2->v;
    out->w = p1->w - p2->w;
    return EMCPOSE_ERR_OK;

}

int emcPoseSelfSub(EmcPose * const self, EmcPose const * const p2)
{
    return emcPoseSub(self, p2, self);
}


int pmCartesianToEmcPose(PmCartesian const * const xyz,
        PmCartesian const * const abc, PmCartesian const * const uvw, EmcPose * const pose)
{
#ifdef EMCPOSE_PEDANTIC
    if (!pose) {
        return EMCPOSE_ERR_OUTPUT_MISSING;
    }
    if (!xyz || !abc || !uvw) {
        return EMCPOSE_ERR_INPUT_MISSING;
    }
#endif
    //Direct copy of translation struct for xyz
    pose->tran = *xyz;

    pose->a = abc->x;
    pose->b = abc->y;
    pose->c = abc->z;

    pose->u = uvw->x;
    pose->v = uvw->y;
    pose->w = uvw->z;
    return EMCPOSE_ERR_OK;
}

int emcPoseToPmCartesian(EmcPose const * const pose,
        PmCartesian * const xyz, PmCartesian * const abc, PmCartesian * const uvw)
{

#ifdef EMCPOSE_PEDANTIC
    if (!pose) {
        return EMCPOSE_ERR_INPUT_MISSING;
    }
    if (!xyz | !abc || !uvw) {
        return EMCPOSE_ERR_OUTPUT_MISSING;
    }
#endif

    //Direct copy of translation struct for xyz
    *xyz = pose->tran;

    //Convert ABCUVW axes into 2 pairs of 3D lines
    abc->x = pose->a;
    abc->y = pose->b;
    abc->z = pose->c;

    uvw->x = pose->u;
    uvw->y = pose->v;
    uvw->z = pose->w;
    return EMCPOSE_ERR_OK;
}

int emcPoseGetXYZ(EmcPose const * const pose, PmCartesian * const xyz)
{
#ifdef EMCPOSE_PEDANTIC
    if (!pose) {
        return EMCPOSE_ERR_OUTPUT_MISSING;
    }
    if (!xyz) {
        return EMCPOSE_ERR_INPUT_MISSING;
    }
#endif

    xyz->x = pose->tran.x;
    xyz->y = pose->tran.y;
    xyz->z = pose->tran.z;
    return EMCPOSE_ERR_OK;
}