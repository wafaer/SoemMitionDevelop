//
// Created by Administrator on 2025/8/19.
//
#include "rtapi/rtapi_math.h"
#include "rtapi/rtapi.h"
#include "posemath.h"

#define IS_FUZZ(a,fuzz) (fabs(a) < (fuzz))

int pmClose(double a, double b, double eps) { return ((fabs((a) - (b)) < (eps)) ? 1 : 0); }

int pmErrno = PM_OK;
double pmSqrt(double x)
{
    if (x > 0.0) {
        pmErrno = PM_OK;
        return sqrt(x);
    }

    if (x > SQRT_FUZZ) {
        pmErrno = PM_OK;
        return 0.0;
    }

    pmErrno = PM_ERR;
    return 0.0;
}

int pmCartCartSub(PmCartesian const * const v1, PmCartesian const * const v2,
        PmCartesian * const vout)
{
    vout->x = v1->x - v2->x;
    vout->y = v1->y - v2->y;
    vout->z = v1->z - v2->z;

    return pmErrno = PM_OK;
}

int pmCartMag(PmCartesian const * const v, double *d)
{
    *d = pmSqrt(pmSq(v->x) + pmSq(v->y) + pmSq(v->z));

    return pmErrno = PM_OK;
}

int pmCartMagSq(PmCartesian const * const v, double *d)
{
    *d = pmSq(v->x) + pmSq(v->y) + pmSq(v->z);

    return pmErrno = PM_OK;
}

int pmCartInfNorm(PmCartesian const * v, double * out)
{
    *out = fmax(fabs(v->x),fmax(fabs(v->y),fabs(v->z)));

    return pmErrno = PM_OK;
}

int pmCartUnitEq(PmCartesian * const v)
{
    double size = pmSqrt(pmSq(v->x) + pmSq(v->y) + pmSq(v->z));

    if (size == 0.0) {
        return pmErrno = PM_NORM_ERR;
    }

    v->x /= size;
    v->y /= size;
    v->z /= size;

    return pmErrno = PM_OK;
}

int pmCartLineInit(PmCartLine * const line, PmCartesian const * const start, PmCartesian const * const end)
{
    int r1 = PM_OK, r2 = PM_OK;

    if (0 == line) {
        return pmErrno = PM_ERR;
    }

    line->start = *start;
    line->end = *end;
    r1 = pmCartCartSub(end, start, &line->uVec);
    if (r1) {
        pmErrno = PM_NORM_ERR;
        return r1;
    }

    pmCartMag(&line->uVec, &line->tmag);
    // NOTE: use the same criteria for "zero" length vectors as used by canon
    double max_xyz=0.0;
    pmCartInfNorm(&line->uVec, &max_xyz);

    if (IS_FUZZ(max_xyz, CART_FUZZ)) {
        line->uVec.x = 1.0;
        line->uVec.y = 0.0;
        line->uVec.z = 0.0;
        line->tmag_zero = 1;
    } else {
        r2 = pmCartUnitEq(&line->uVec);
        line->tmag_zero = 0;
    }

    /* return PM_NORM_ERR if uVec has been set to 1, 0, 0 */
    return pmErrno = (r1 || r2) ? PM_NORM_ERR : PM_OK;
}

int pmCartScalMult(PmCartesian const * const v1, double d, PmCartesian * const vout)
{
    if (v1 != vout) {
        *vout = *v1;
    }

    return pmCartScalMultEq(vout, d);
}

int pmCartScalMultEq(PmCartesian * const v, double d)
{

    v->x *= d;
    v->y *= d;
    v->z *= d;

    return pmErrno = PM_OK;
}

int pmCartLinePoint(PmCartLine const * const line, double len, PmCartesian * const point)
{
    int r1 = PM_OK, r2 = PM_OK;

    if (line->tmag_zero) {
        *point = line->end;
    } else {
        /* return start + len * uVec */
        r1 = pmCartScalMult(&line->uVec, len, point);
        r2 = pmCartCartAdd(&line->start, point, point);
    }

    return pmErrno = (r1 || r2) ? PM_NORM_ERR : PM_OK;
}

int pmCartCartAdd(PmCartesian const * const v1, PmCartesian const * const v2, PmCartesian * const vout)
{
    double x = v1->x + v2->x;
    double y = v1->y + v2->y;
    double z = v1->z + v2->z;

    vout->x = x;
    vout->y = y;
    vout->z = z;

    return pmErrno = PM_OK;
}

int pmCartCartAddEq(PmCartesian * const v, PmCartesian const * const v_add)
{
    v->x += v_add->x;
    v->y += v_add->y;
    v->z += v_add->z;

    return pmErrno = PM_OK;
}

int pmCartLineStretch(PmCartLine * const line, double new_len, int from_end)
{
    int r1 = PM_OK, r2 = PM_OK;

    if (!line || line->tmag_zero || new_len <= DOUBLE_FUZZ) {
        return pmErrno = PM_ERR;
    }

    if (from_end) {
        // Store the new relative position from end in the start point
        r1 = pmCartScalMult(&line->uVec, -new_len, &line->start);
        // Offset the new start point by the current end point
        r2 = pmCartCartAddEq(&line->start, &line->end);
    } else {
        // Store the new relative position from start in the end point:
        r1 = pmCartScalMult(&line->uVec, new_len, &line->end);
        // Offset the new end point by the current start point
        r2 = pmCartCartAdd(&line->start, &line->end, &line->end);
    }
    line->tmag = new_len;

    return pmErrno = (r1 || r2) ? PM_NORM_ERR : PM_OK;
}

int pmCylCartConvert(PmCylindrical const * const c, PmCartesian * const v)
{
    v->x = c->r * cos(c->theta);
    v->y = c->r * sin(c->theta);
    v->z = c->z;

    return pmErrno = PM_OK;
}


int pmCartCartDot(PmCartesian const * const v1, PmCartesian const * const v2, double *d)
{
    *d = v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;

    return pmErrno = PM_OK;
}

int pmCartCartProj(PmCartesian const * const v1, PmCartesian const * const v2, PmCartesian * const vout)
{
    int r1, r2;
    int r3=1;
    double d12;
    double d22;

    r1 = pmCartCartDot(v1, v2, &d12);
    r2 = pmCartCartDot(v2, v2, &d22);
    if (!(r1 || r2)){
        r3 = pmCartScalMult(v2, d12/d22, vout);
    }

    return pmErrno = (r1 || r2 || r3) ? PM_NORM_ERR : PM_OK;
}

int pmCartUnit(PmCartesian const * const v, PmCartesian * const vout)
{
    if (vout != v) {
        *vout = *v;
    }
    return pmCartUnitEq(vout);
}


int pmCartCartDisp(PmCartesian const * const v1, PmCartesian const * const v2,
        double *d)
{
    *d = pmSqrt(pmSq(v2->x - v1->x) + pmSq(v2->y - v1->y) + pmSq(v2->z - v1->z));

    return pmErrno = PM_OK;
}

int pmCartCartCross(PmCartesian const * const v1, PmCartesian const * const v2,
        PmCartesian * const vout)
{
    if (vout == v1 || vout == v2) {
        return pmErrno = PM_IMPL_ERR;
    }
    vout->x = v1->y * v2->z - v1->z * v2->y;
    vout->y = v1->z * v2->x - v1->x * v2->z;
    vout->z = v1->x * v2->y - v1->y * v2->x;

    return pmErrno = PM_OK;
}

int pmCartPlaneProj(PmCartesian const * const v, PmCartesian const * const normal, PmCartesian * const vout)
{
    int r1, r2;
    PmCartesian par;

    r1 = pmCartCartProj(v, normal, &par);
    r2 = pmCartCartSub(v, &par, vout);

    return pmErrno = (r1 || r2) ? PM_NORM_ERR : PM_OK;
}

int pmCircleInit(PmCircle * const circle,
        PmCartesian const * const start, PmCartesian const * const end,
        PmCartesian const * const center, PmCartesian const * const normal, int turn)
{
    double dot;
    PmCartesian rEnd;
    PmCartesian v;
    double d;
    int r1;
    PmCartesian p1;
    PmCartesian p2;

    /* adjust center */
    pmCartCartSub(start, center, &v);
    r1 = pmCartCartProj(&v, normal, &v);
    if (PM_NORM_ERR == r1) {
        return pmErrno = PM_ERR;
    }
    pmCartCartAdd(&v, center, &circle->center);

    /* normalize and redirect normal vector based on turns. If turn is less
       than 0, point normal vector in other direction and make turn positive,
       -1 -> 0, -2 -> 1, etc. */
    pmCartUnit(normal, &circle->normal);
    if (turn < 0) {
        turn = -1 - turn;
        pmCartScalMult(&circle->normal, -1.0, &circle->normal);
    }

    /* radius */
    pmCartCartDisp(start, &circle->center, &circle->radius);

    /* vector in plane of circle from center to start, magnitude radius */
    pmCartCartSub(start, &circle->center, &circle->rTan);
    /* vector in plane of circle perpendicular to rTan, magnitude radius */
    pmCartCartCross(&circle->normal, &circle->rTan, &circle->rPerp);

    /* do rHelix, rEnd */
    pmCartCartSub(end, &circle->center, &circle->rHelix);
    pmCartPlaneProj(&circle->rHelix, &circle->normal, &rEnd);
    pmCartMag(&rEnd, &circle->spiral);
    circle->spiral -= circle->radius;
    pmCartCartSub(&circle->rHelix, &rEnd, &circle->rHelix);
    pmCartUnit(&rEnd, &rEnd);
    pmCartScalMult(&rEnd, circle->radius, &rEnd);


    pmCartMag(&rEnd, &d);
    if (d == 0.0) {
        pmCartScalMult(&circle->normal, DOUBLE_FUZZ, &v);
        pmCartCartAdd(&rEnd, &v, &rEnd);
    }

    pmCartCartDot(&circle->rTan, &rEnd, &dot);
    dot = dot / (circle->radius * circle->radius);
    if (dot > 1.0) {
        circle->angle = 0.0;
    } else if (dot < -1.0) {
        circle->angle = PM_PI;
    } else {
        circle->angle = acos(dot);
    }

    pmCartCartCross(&circle->rTan, &rEnd, &v);
    pmCartCartDot(&v, &circle->normal, &d);
    if (d < CART_FUZZ) {
        circle->angle = PM_2_PI - circle->angle;
    }

    pmCartPlaneProj(start, normal, &p1);
    pmCartPlaneProj(end, normal, &p2);
    pmCartCartDisp(&p1, &p2, &d);
    if (d < CART_FUZZ){
        circle->angle = PM_2_PI;
    }

    /* now add more angle for multi turns */
    if (turn > 0) {
        circle->angle += turn * 2.0 * PM_PI;
    }

    /* FIXME: some code has an unguarded division by circle->angle */
    if (circle->angle == 0) circle->angle += CIRCLE_FUZZ / 2;

    return pmErrno = PM_OK;
}

int pmCirclePoint(PmCircle const * const circle, double angle, PmCartesian * const point)
{
    PmCartesian par, perp;
    double scale;

    /* compute components rel to center */
    pmCartScalMult(&circle->rTan, cos(angle), &par);
    pmCartScalMult(&circle->rPerp, sin(angle), &perp);

    /* add to get radius vector rel to center */
    pmCartCartAdd(&par, &perp, point);

    /* get scale for spiral, helix interpolation */
    if (circle->angle == 0.0) {
	return pmErrno = PM_DIV_ERR;
    }
    scale = angle / circle->angle;

    /* add scaled vector in radial dir for spiral */
    pmCartUnit(point, &par);
    pmCartScalMult(&par, scale * circle->spiral, &par);
    pmCartCartAdd(point, &par, point);

    /* add scaled vector in helix dir */
    pmCartScalMult(&circle->rHelix, scale, &perp);
    pmCartCartAdd(point, &perp, point);

    /* add to center vector for final result */
    pmCartCartAdd(&circle->center, point, point);

    return pmErrno = PM_OK;
}

int pmCircleStretch(PmCircle * const circ, double new_angle, int from_end)
{
    if (!circ || new_angle <= DOUBLE_FUZZ) {
        return pmErrno = PM_ERR;
    }

    double mag = 0;
    pmCartMagSq(&circ->rHelix, &mag);
    if ( mag > 1e-6 ) {
        //Can't handle helices
        return pmErrno = PM_ERR;
    }
    if (from_end) {
        //Not implemented yet, way more reprocessing...
        PmCartesian new_start;
        double start_angle = circ->angle - new_angle;
        pmCirclePoint(circ, start_angle, &new_start);
        pmCartCartSub(&new_start, &circ->center, &circ->rTan);
        pmCartCartCross(&circ->normal, &circ->rTan, &circ->rPerp);
        pmCartMag(&circ->rTan, &circ->radius);
    }
    //Reduce the spiral proportionally
    circ->spiral *= (new_angle / circ->angle);
    // Easy to grow / shrink from start
    circ->angle = new_angle;

    return pmErrno = PM_OK;
}

int pmRotNorm(PmRotationVector const * const r, PmRotationVector * const rout)
{
    double size;

    size = pmSqrt(pmSq(r->x) + pmSq(r->y) + pmSq(r->z));

    if (fabs(r->s) < RS_FUZZ) {
        rout->s = 0.0;
        rout->x = 0.0;
        rout->y = 0.0;
        rout->z = 0.0;

        return pmErrno = PM_OK;
    }

    if (size == 0.0) {

        rout->s = 0.0;
        rout->x = 0.0;
        rout->y = 0.0;
        rout->z = 0.0;

        return pmErrno = PM_NORM_ERR;
    }

    rout->s = r->s;
    rout->x = r->x / size;
    rout->y = r->y / size;
    rout->z = r->z / size;

    return pmErrno = PM_OK;
}

int pmQuatInv(PmQuaternion const * const q1, PmQuaternion * const qout)
{
    if (qout == 0) {
        return pmErrno = PM_ERR;
    }

    qout->s = q1->s;
    qout->x = -q1->x;
    qout->y = -q1->y;
    qout->z = -q1->z;

    return pmErrno = PM_OK;
}

int pmQuatQuatMult(PmQuaternion const * const q1, PmQuaternion const * const q2, PmQuaternion * const qout)
{
    if (qout == 0) {
        return pmErrno = PM_ERR;
    }

    qout->s = q1->s * q2->s - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;

    if (qout->s >= 0.0) {
        qout->x = q1->s * q2->x + q1->x * q2->s + q1->y * q2->z - q1->z * q2->y;
        qout->y = q1->s * q2->y - q1->x * q2->z + q1->y * q2->s + q1->z * q2->x;
        qout->z = q1->s * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->s;
    } else {
        qout->s *= -1;
        qout->x = -q1->s * q2->x - q1->x * q2->s - q1->y * q2->z + q1->z * q2->y;
        qout->y = -q1->s * q2->y + q1->x * q2->z - q1->y * q2->s - q1->z * q2->x;
        qout->z = -q1->s * q2->z - q1->x * q2->y + q1->y * q2->x - q1->z * q2->s;
    }


    return pmErrno = PM_OK;
}

int pmQuatRotConvert(PmQuaternion const * const q, PmRotationVector * const r)
{
    double sh;

    sh = pmSqrt(pmSq(q->x) + pmSq(q->y) + pmSq(q->z));

    if (sh > QSIN_FUZZ) {
        r->s = 2.0 * atan2(sh, q->s);
        r->x = q->x / sh;
        r->y = q->y / sh;
        r->z = q->z / sh;
    } else {
        r->s = 0.0;
        r->x = 0.0;
        r->y = 0.0;
        r->z = 0.0;
    }

    return pmErrno = PM_OK;
}

int pmQuatMag(PmQuaternion const * const q, double *d)
{
    PmRotationVector r;
    int r1;

    if (0 == d) {
        return pmErrno = PM_ERR;
    }

    r1 = pmQuatRotConvert(q, &r);
    *d = r.s;

    return pmErrno = r1;
}

int pmRotScalMult(PmRotationVector const * const r, double s, PmRotationVector * const rout)
{
    rout->s = r->s * s;
    rout->x = r->x;
    rout->y = r->y;
    rout->z = r->z;

    return pmErrno = PM_OK;
}

void pm_sincos(double x, double *sx, double *cx)
{
    *sx = sin(x);
    *cx = cos(x);
}

int pmRotQuatConvert(PmRotationVector const * const r, PmQuaternion * const q)
{
    double sh;

    if (pmClose(r->s, 0.0, QS_FUZZ)) {
        q->s = 1.0;
        q->x = q->y = q->z = 0.0;

        return pmErrno = PM_OK;
    }

    pm_sincos(r->s / 2.0, &sh, &(q->s));

    if (q->s >= 0.0) {
        q->x = r->x * sh;
        q->y = r->y * sh;
        q->z = r->z * sh;
    } else {
        q->s *= -1;
        q->x = -r->x * sh;
        q->y = -r->y * sh;
        q->z = -r->z * sh;
    }

    return pmErrno = PM_OK;
}

int pmQuatScalMult(PmQuaternion const * const q, double s, PmQuaternion * const qout)
{
    PmRotationVector r;
    int r1, r2, r3;

    r1 = pmQuatRotConvert(q, &r);
    r2 = pmRotScalMult(&r, s, &r);
    r3 = pmRotQuatConvert(&r, qout);

    return pmErrno = (r1 || r2 || r3) ? PM_NORM_ERR : PM_OK;
}

int pmLineInit(PmLine * const line, PmPose const * const start, PmPose const * const end)
{
    int r1, r2, r3, r4, r5;
    r1 = r2 = r3 = r4 = r5 = PM_OK;

    double tmag = 0.0;
    double rmag = 0.0;
    PmQuaternion startQuatInverse;

    if (0 == line) {
        return pmErrno = PM_ERR;
    }

    r3 = pmQuatInv(&start->rot, &startQuatInverse);
    if (r3) {
        pmErrno = PM_NORM_ERR;
        return r3;
    }

    r4 = pmQuatQuatMult(&startQuatInverse, &end->rot, &line->qVec);
    if (r4) {
        pmErrno = PM_NORM_ERR;
        return r4;
    }

    pmQuatMag(&line->qVec, &rmag);
    if (rmag > Q_FUZZ) {
        r5 = pmQuatScalMult(&line->qVec, 1 / rmag, &(line->qVec));
        if (r5) {
            pmErrno = PM_NORM_ERR;
            return r5;
        }
    }

    line->start = *start;
    line->end = *end;
    r1 = pmCartCartSub(&end->tran, &start->tran, &line->uVec);
    if (r1) {
        pmErrno = PM_NORM_ERR;
        return r1;
    }

    pmCartMag(&line->uVec, &tmag);
    if (IS_FUZZ(tmag, CART_FUZZ)) {
        line->uVec.x = 1.0;
        line->uVec.y = 0.0;
        line->uVec.z = 0.0;
    } else {
        r2 = pmCartUnit(&line->uVec, &line->uVec);
    }
    line->tmag = tmag;
    line->rmag = rmag;
    line->tmag_zero = (line->tmag <= CART_FUZZ);
    line->rmag_zero = (line->rmag <= Q_FUZZ);

    /* return PM_NORM_ERR if uVec has been set to 1, 0, 0 */
    return pmErrno = (r1 || r2 || r3 || r4 || r5) ? PM_NORM_ERR : PM_OK;
}

int pmLinePoint(PmLine const * const line, double len, PmPose * const point)
{
    int r1, r2, r3, r4;
    r1 = r2 = r3 = r4 = PM_OK;

    if (line->tmag_zero) {
        point->tran = line->end.tran;
    } else {
        /* return start + len * uVec */
        r1 = pmCartScalMult(&line->uVec, len, &point->tran);
        r2 = pmCartCartAdd(&line->start.tran, &point->tran, &point->tran);
    }

    if (line->rmag_zero) {
        point->rot = line->end.rot;
    } else {
        if (line->tmag_zero) {
            r3 = pmQuatScalMult(&line->qVec, len, &point->rot);
        } else {
            r3 = pmQuatScalMult(&line->qVec, len * line->rmag / line->tmag,
            &point->rot);
        }
        r4 = pmQuatQuatMult(&line->start.rot, &point->rot, &point->rot);
    }

    return pmErrno = (r1 || r2 || r3 || r4) ? PM_NORM_ERR : PM_OK;
}

int pmCartIsNorm(PmCartesian const * const v)
{
    return pmSqrt(pmSq(v->x) + pmSq(v->y) + pmSq(v->z)) - 1.0 < UNIT_VEC_FUZZ;
}

int pmQuatIsNorm(PmQuaternion const * const q1)
{
    return (fabs(pmSq(q1->s) + pmSq(q1->x) + pmSq(q1->y) + pmSq(q1->z) - 1.0) <
    UNIT_QUAT_FUZZ);
}

int pmRotIsNorm(PmRotationVector const * const r)
{
    if (fabs(r->s) < RS_FUZZ ||
    fabs(pmSqrt(pmSq(r->x) + pmSq(r->y) + pmSq(r->z))) - 1.0 < UNIT_VEC_FUZZ)
    {
        return 1;
    }

    return 0;
}

int pmMatIsNorm(PmRotationMatrix const * const m)
{
    PmCartesian u;

    pmCartCartCross(&m->x, &m->y, &u);

    return (pmCartIsNorm(&m->x) && pmCartIsNorm(&m->y) && pmCartIsNorm(&m->z) && pmCartCartCompare(&u, &m->z));
}

int pmCartCartCompare(PmCartesian const * const v1, PmCartesian const * const v2)
{
    if (fabs(v1->x - v2->x) >= V_FUZZ ||
    fabs(v1->y - v2->y) >= V_FUZZ || fabs(v1->z - v2->z) >= V_FUZZ) {
        return 0;
    }

    return 1;
}

int pmCartInvEq(PmCartesian * const v)
{
    double size_sq;
    pmCartMagSq(v,&size_sq);

    v->x /= size_sq;
    v->y /= size_sq;
    v->z /= size_sq;

    return pmErrno = PM_OK;
}

int pmCartInv(PmCartesian const * const v1, PmCartesian * const vout)
{
    if (v1 != vout) {
        *vout = *v1;
    }

    return pmCartInvEq(vout);
}

int pmMatInv(PmRotationMatrix const * const m, PmRotationMatrix * const mout)
{
    /* inverse of a rotation matrix is the transpose */

    mout->x.x = m->x.x;
    mout->x.y = m->y.x;
    mout->x.z = m->z.x;

    mout->y.x = m->x.y;
    mout->y.y = m->y.y;
    mout->y.z = m->z.y;

    mout->z.x = m->x.z;
    mout->z.y = m->y.z;
    mout->z.z = m->z.z;

    return pmErrno = PM_OK;
}

int pmPoseInv(PmPose const * const p1, PmPose * const p2)
{
    int r1, r2;

    r1 = pmQuatInv(&p1->rot, &p2->rot);
    r2 = pmQuatCartMult(&p2->rot, &p1->tran, &p2->tran);

    p2->tran.x *= -1.0;
    p2->tran.y *= -1.0;
    p2->tran.z *= -1.0;

    return pmErrno = (r1 || r2) ? PM_NORM_ERR : PM_OK;
}

int pmQuatQuatCompare(PmQuaternion const * const q1, PmQuaternion const * const q2)
{
    if (fabs(q1->s - q2->s) < Q_FUZZ &&
    fabs(q1->x - q2->x) < Q_FUZZ &&
    fabs(q1->y - q2->y) < Q_FUZZ && fabs(q1->z - q2->z) < Q_FUZZ) {
        return 1;
    }

    /* note (0, x, y, z) = (0, -x, -y, -z) */
    if (fabs(q1->s) >= QS_FUZZ ||
    fabs(q1->x + q2->x) >= Q_FUZZ ||
    fabs(q1->y + q2->y) >= Q_FUZZ || fabs(q1->z + q2->z) >= Q_FUZZ) {
        return 0;
    }

    return 1;
}

int pmSphCartConvert(PmSpherical const * const s, PmCartesian * const v)
{
    double _r;

    _r = s->r * sin(s->phi);
    v->z = s->r * cos(s->phi);
    v->x = _r * cos(s->theta);
    v->y = _r * sin(s->theta);

    return pmErrno = PM_OK;
}

int pmCartSphConvert(PmCartesian const * const v, PmSpherical * const s)
{
    double _r;

    s->theta = atan2(v->y, v->x);
    s->r = pmSqrt(pmSq(v->x) + pmSq(v->y) + pmSq(v->z));
    _r = pmSqrt(pmSq(v->x) + pmSq(v->y));
    s->phi = atan2(_r, v->z);

    return pmErrno = PM_OK;
}

int pmCylSphConvert(PmCylindrical const * const c, PmSpherical * const s)
{
    s->theta = c->theta;
    s->r = pmSqrt(pmSq(c->r) + pmSq(c->z));
    s->phi = atan2(c->z, c->r);

    return pmErrno = PM_OK;
}

int pmCartCylConvert(PmCartesian const * const v, PmCylindrical * const c)
{
    c->theta = atan2(v->y, v->x);
    c->r = pmSqrt(pmSq(v->x) + pmSq(v->y));
    c->z = v->z;

    return pmErrno = PM_OK;
}

int pmSphCylConvert(PmSpherical const * const s, PmCylindrical * const c)
{
    c->theta = s->theta;
    c->r = s->r * cos(s->phi);
    c->z = s->r * sin(s->phi);

    return pmErrno = PM_OK;
}

int pmRotMatConvert(PmRotationVector const * const r, PmRotationMatrix * const m)
{
    double s, c, omc;

    pm_sincos(r->s, &s, &c);

    /* from space book */
    m->x.x = c + pmSq(r->x) * (omc = 1 - c);	/* omc = One Minus Cos */
    m->y.x = -r->z * s + r->x * r->y * omc;
    m->z.x = r->y * s + r->x * r->z * omc;

    m->x.y = r->z * s + r->y * r->x * omc;
    m->y.y = c + pmSq(r->y) * omc;
    m->z.y = -r->x * s + r->y * r->z * omc;

    m->x.z = -r->y * s + r->z * r->x * omc;
    m->y.z = r->x * s + r->z * r->y * omc;
    m->z.z = c + pmSq(r->z) * omc;

    return pmErrno = PM_OK;
}

int pmQuatMatConvert(PmQuaternion const * const q, PmRotationMatrix * const m)
{
    /* from space book where e1=q->x e2=q->y e3=q->z e4=q->s */
    m->x.x = 1.0 - 2.0 * (pmSq(q->y) + pmSq(q->z));
    m->y.x = 2.0 * (q->x * q->y - q->z * q->s);
    m->z.x = 2.0 * (q->z * q->x + q->y * q->s);

    m->x.y = 2.0 * (q->x * q->y + q->z * q->s);
    m->y.y = 1.0 - 2.0 * (pmSq(q->z) + pmSq(q->x));
    m->z.y = 2.0 * (q->y * q->z - q->x * q->s);

    m->x.z = 2.0 * (q->z * q->x - q->y * q->s);
    m->y.z = 2.0 * (q->y * q->z + q->x * q->s);
    m->z.z = 1.0 - 2.0 * (pmSq(q->x) + pmSq(q->y));

    return pmErrno = PM_OK;
}

int pmRpyMatConvert(PmRpy  const * const rpy, PmRotationMatrix * const m)
{
    double sa, sb, sg;
    double ca, cb, cg;

    sa = sin(rpy->y);
    sb = sin(rpy->p);
    sg = sin(rpy->r);

    ca = cos(rpy->y);
    cb = cos(rpy->p);
    cg = cos(rpy->r);

    m->x.x = ca * cb;
    m->y.x = ca * sb * sg - sa * cg;
    m->z.x = ca * sb * cg + sa * sg;

    m->x.y = sa * cb;
    m->y.y = sa * sb * sg + ca * cg;
    m->z.y = sa * sb * cg - ca * sg;

    m->x.z = -sb;
    m->y.z = cb * sg;
    m->z.z = cb * cg;

    return pmErrno = PM_OK;
}

int pmZyzMatConvert(PmEulerZyz  const * const zyz, PmRotationMatrix * const m)
{
    double sa, sb, sg;
    double ca, cb, cg;

    sa = sin(zyz->z);
    sb = sin(zyz->y);
    sg = sin(zyz->zp);

    ca = cos(zyz->z);
    cb = cos(zyz->y);
    cg = cos(zyz->zp);

    m->x.x = ca * cb * cg - sa * sg;
    m->y.x = -ca * cb * sg - sa * cg;
    m->z.x = ca * sb;

    m->x.y = sa * cb * cg + ca * sg;
    m->y.y = -sa * cb * sg + ca * cg;
    m->z.y = sa * sb;

    m->x.z = -sb * cg;
    m->y.z = sb * sg;
    m->z.z = cb;

    return pmErrno = PM_OK;
}

int pmZyxMatConvert(PmEulerZyx  const * const zyx, PmRotationMatrix * const m)
{
    double sa, sb, sg;
    double ca, cb, cg;

    sa = sin(zyx->z);
    sb = sin(zyx->y);
    sg = sin(zyx->x);

    ca = cos(zyx->z);
    cb = cos(zyx->y);
    cg = cos(zyx->x);

    m->x.x = ca * cb;
    m->y.x = ca * sb * sg - sa * cg;
    m->z.x = ca * sb * cg + sa * sg;

    m->x.y = sa * cb;
    m->y.y = sa * sb * sg + ca * cg;
    m->z.y = sa * sb * cg - ca * sg;

    m->x.z = -sb;
    m->y.z = cb * sg;
    m->z.z = cb * cg;

    return pmErrno = PM_OK;
}

int pmQuatNorm(PmQuaternion const * const q1, PmQuaternion * const qout)
{
    double size = pmSqrt(pmSq(q1->s) + pmSq(q1->x) + pmSq(q1->y) + pmSq(q1->z));

    if (size == 0.0) {
        qout->s = 1;
        qout->x = 0;
        qout->y = 0;
        qout->z = 0;

        return pmErrno = PM_NORM_ERR;
    }

    if (q1->s >= 0.0) {
        qout->s = q1->s / size;
        qout->x = q1->x / size;
        qout->y = q1->y / size;
        qout->z = q1->z / size;

        return pmErrno = PM_OK;
    } else {
        qout->s = -q1->s / size;
        qout->x = -q1->x / size;
        qout->y = -q1->y / size;
        qout->z = -q1->z / size;

        return pmErrno = PM_OK;
    }
}

int pmMatQuatConvert(PmRotationMatrix const * const m, PmQuaternion * const q)
{
    double a;

    q->s = 0.5 * pmSqrt(1.0 + m->x.x + m->y.y + m->z.z);

    if (fabs(q->s) > QS_FUZZ) {
	q->x = (m->y.z - m->z.y) / (a = 4 * q->s);
	q->y = (m->z.x - m->x.z) / a;
	q->z = (m->x.y - m->y.x) / a;
    } else {
	q->s = 0;
	q->x = pmSqrt(1.0 + m->x.x - m->y.y - m->z.z) / 2.0;
	q->y = pmSqrt(1.0 + m->y.y - m->x.x - m->z.z) / 2.0;
	q->z = pmSqrt(1.0 + m->z.z - m->y.y - m->x.x) / 2.0;

	if (q->x > q->y && q->x > q->z) {
	    if (m->x.y < 0.0) {
		q->y *= -1;
	    }
	    if (m->x.z < 0.0) {
		q->z *= -1;
	    }
	} else if (q->y > q->z) {
	    if (m->x.y < 0.0) {
		q->x *= -1;
	    }
	    if (m->y.z < 0.0) {
		q->z *= -1;
	    }
	} else {
	    if (m->x.z < 0.0) {
		q->x *= -1;
	    }
	    if (m->y.z < 0.0) {
		q->y *= -1;
	    }
	}
    }

    pmErrno = PM_OK;
    return pmQuatNorm(q, q);
}

int pmZyzQuatConvert(PmEulerZyz const * const zyz, PmQuaternion * const q)
{
    PmRotationMatrix m;
    int r1, r2;

    r1 = pmZyzMatConvert(zyz, &m);
    r2 = pmMatQuatConvert(&m, q);

    return pmErrno = (r1 || r2) ? PM_NORM_ERR : PM_OK;
}

int pmZyxQuatConvert(PmEulerZyx  const * const zyx, PmQuaternion * const q)
{
    PmRotationMatrix m;
    int r1, r2;

    r1 = pmZyxMatConvert(zyx, &m);
    r2 = pmMatQuatConvert(&m, q);

    return pmErrno = (r1 || r2) ? PM_NORM_ERR : PM_OK;
}

int pmRpyQuatConvert(PmRpy  const * const rpy, PmQuaternion * const q)
{
    PmRotationMatrix m;
    int r1, r2;

    r1 = pmRpyMatConvert(rpy, &m);
    r2 = pmMatQuatConvert(&m, q);

    return pmErrno = (r1 || r2) ? PM_NORM_ERR : PM_OK;
}

int pmAxisAngleQuatConvert(PmAxis axis, double a, PmQuaternion * const q)
{
    double sh;

    a *= 0.5;
    pm_sincos(a, &sh, &(q->s));

    switch (axis) {
        case PM_X:
            q->x = sh;
        q->y = 0.0;
        q->z = 0.0;
        break;

        case PM_Y:
            q->x = 0.0;
        q->y = sh;
        q->z = 0.0;
        break;

        case PM_Z:
            q->x = 0.0;
        q->y = 0.0;
        q->z = sh;
        break;

        default:
        return pmErrno = PM_ERR;
    }

    if (q->s < 0.0) {
        q->s *= -1.0;
        q->x *= -1.0;
        q->y *= -1.0;
        q->z *= -1.0;
    }

    return pmErrno = PM_OK;
}

int pmQuatAxisAngleMult(PmQuaternion const * const q, PmAxis axis, double angle,
    PmQuaternion * const pq)
{
    double sh, ch;

    angle *= 0.5;
    pm_sincos(angle, &sh, &ch);

    switch (axis) {
        case PM_X:
            pq->s = ch * q->s - sh * q->x;
        pq->x = ch * q->x + sh * q->s;
        pq->y = ch * q->y + sh * q->z;
        pq->z = ch * q->z - sh * q->y;
        break;

        case PM_Y:
            pq->s = ch * q->s - sh * q->y;
        pq->x = ch * q->x - sh * q->z;
        pq->y = ch * q->y + sh * q->s;
        pq->z = ch * q->z + sh * q->x;
        break;

        case PM_Z:
            pq->s = ch * q->s - sh * q->z;
        pq->x = ch * q->x + sh * q->y;
        pq->y = ch * q->y - sh * q->x;
        pq->z = ch * q->z + sh * q->s;
        break;

        default:
        return pmErrno = PM_ERR;
    }

    if (pq->s < 0.0) {
        pq->s *= -1.0;
        pq->x *= -1.0;
        pq->y *= -1.0;
        pq->z *= -1.0;
    }

    return 0;
}

int pmMatZyzConvert(PmRotationMatrix const * const m, PmEulerZyz * const zyz)
{
    zyz->y = atan2(pmSqrt(pmSq(m->x.z) + pmSq(m->y.z)), m->z.z);

    if (fabs(zyz->y) < ZYZ_Y_FUZZ) {
        zyz->z = 0.0;
        zyz->y = 0.0;		/* force Y to 0 */
        zyz->zp = atan2(-m->y.x, m->x.x);
    } else if (fabs(zyz->y - PM_PI) < ZYZ_Y_FUZZ) {
        zyz->z = 0.0;
        zyz->y = PM_PI;		/* force Y to 180 */
        zyz->zp = atan2(m->y.x, -m->x.x);
    } else {
        zyz->z = atan2(m->z.y, m->z.x);
        zyz->zp = atan2(m->y.z, -m->x.z);
    }

    return pmErrno = PM_OK;
}

int pmQuatZyzConvert(PmQuaternion const * const q, PmEulerZyz * const zyz)
{
    PmRotationMatrix m;
    int r1, r2;

    r1 = pmQuatMatConvert(q, &m);
    r2 = pmMatZyzConvert(&m, zyz);

    return pmErrno = (r1 || r2) ? PM_NORM_ERR : PM_OK;
}

int pmQuatZyxConvert(PmQuaternion const * const q, PmEulerZyx * const zyx)
{
    PmRotationMatrix m;
    int r1, r2;

    r1 = pmQuatMatConvert(q, &m);
    r2 = pmMatZyxConvert(&m, zyx);

    return pmErrno = (r1 || r2) ? PM_NORM_ERR : PM_OK;
}

int pmMatZyxConvert(PmRotationMatrix const * const m, PmEulerZyx * const zyx)
{
    zyx->y = atan2(-m->x.z, pmSqrt(pmSq(m->x.x) + pmSq(m->x.y)));

    if (fabs(zyx->y - PM_PI_2) < ZYX_Y_FUZZ) {
        zyx->z = 0.0;
        zyx->y = PM_PI_2;	/* force it */
        zyx->x = atan2(m->y.x, m->y.y);
    } else if (fabs(zyx->y + PM_PI_2) < ZYX_Y_FUZZ) {
        zyx->z = 0.0;
        zyx->y = -PM_PI_2;	/* force it */
        zyx->x = -atan2(m->y.z, m->y.y);
    } else {
        zyx->z = atan2(m->x.y, m->x.x);
        zyx->x = atan2(m->y.z, m->z.z);
    }

    return pmErrno = PM_OK;
}

int pmMatRpyConvert(PmRotationMatrix const * const m, PmRpy * const rpy)
{
    rpy->p = atan2(-m->x.z, pmSqrt(pmSq(m->x.x) + pmSq(m->x.y)));

    if (fabs(rpy->p - PM_PI_2) < RPY_P_FUZZ) {
        rpy->r = atan2(m->y.x, m->y.y);
        rpy->p = PM_PI_2;	/* force it */
        rpy->y = 0.0;
    } else if (fabs(rpy->p + PM_PI_2) < RPY_P_FUZZ) {
        rpy->r = -atan2(m->y.x, m->y.y);
        rpy->p = -PM_PI_2;	/* force it */
        rpy->y = 0.0;
    } else {
        rpy->r = atan2(m->y.z, m->z.z);
        rpy->y = atan2(m->x.y, m->x.x);
    }

    return pmErrno = PM_OK;
}

int pmQuatRpyConvert(PmQuaternion const * const q, PmRpy * const rpy)
{
    PmRotationMatrix m;
    int r1, r2;

    r1 = pmQuatMatConvert(q, &m);
    r2 = pmMatRpyConvert(&m, rpy);

    return pmErrno = (r1 || r2) ? PM_NORM_ERR : PM_OK;
}

int pmHomPoseConvert(PmHomogeneous const * const h, PmPose * const p)
{
    int r1;

    p->tran = h->tran;
    r1 = pmMatQuatConvert(&h->rot, &p->rot);

    return pmErrno = r1;
}

int pmPoseHomConvert(PmPose const * const p, PmHomogeneous * const h)
{
    int r1;

    h->tran = p->tran;
    r1 = pmQuatMatConvert(&p->rot, &h->rot);

    return pmErrno = r1;
}

int pmPosePoseCompare(PmPose const * const p1, PmPose const * const p2)
{
    return pmQuatQuatCompare(&p1->rot, &p2->rot) && pmCartCartCompare(&p1->tran, &p2->tran);
}

int pmQuatCartMult(PmQuaternion const * const q1, PmCartesian const * const v2, PmCartesian * const vout)
{
    PmCartesian c;

    c.x = q1->y * v2->z - q1->z * v2->y;
    c.y = q1->z * v2->x - q1->x * v2->z;
    c.z = q1->x * v2->y - q1->y * v2->x;

    vout->x = v2->x + 2.0 * (q1->s * c.x + q1->y * c.z - q1->z * c.y);
    vout->y = v2->y + 2.0 * (q1->s * c.y + q1->z * c.x - q1->x * c.z);
    vout->z = v2->z + 2.0 * (q1->s * c.z + q1->x * c.y - q1->y * c.x);

    return pmErrno = PM_OK;
}

int pmMatMatMult(PmRotationMatrix const * const m1, PmRotationMatrix const * const m2,
    PmRotationMatrix * const mout)
{
    mout->x.x = m1->x.x * m2->x.x + m1->y.x * m2->x.y + m1->z.x * m2->x.z;
    mout->x.y = m1->x.y * m2->x.x + m1->y.y * m2->x.y + m1->z.y * m2->x.z;
    mout->x.z = m1->x.z * m2->x.x + m1->y.z * m2->x.y + m1->z.z * m2->x.z;

    mout->y.x = m1->x.x * m2->y.x + m1->y.x * m2->y.y + m1->z.x * m2->y.z;
    mout->y.y = m1->x.y * m2->y.x + m1->y.y * m2->y.y + m1->z.y * m2->y.z;
    mout->y.z = m1->x.z * m2->y.x + m1->y.z * m2->y.y + m1->z.z * m2->y.z;

    mout->z.x = m1->x.x * m2->z.x + m1->y.x * m2->z.y + m1->z.x * m2->z.z;
    mout->z.y = m1->x.y * m2->z.x + m1->y.y * m2->z.y + m1->z.y * m2->z.z;
    mout->z.z = m1->x.z * m2->z.x + m1->y.z * m2->z.y + m1->z.z * m2->z.z;

    return pmErrno = PM_OK;
}

int pmPosePoseMult(PmPose const * const p1, PmPose const * const p2, PmPose * const pout)
{
    int r1, r2, r3;

    r1 = pmQuatCartMult(&p1->rot, &p2->tran, &pout->tran);
    r2 = pmCartCartAdd(&p1->tran, &pout->tran, &pout->tran);
    r3 = pmQuatQuatMult(&p1->rot, &p2->rot, &pout->rot);

    return pmErrno = (r1 || r2 || r3) ? PM_NORM_ERR : PM_OK;
}

int pmPoseCartMult(PmPose const * const p1, PmCartesian const * const v2, PmCartesian * const vout)
{
    int r1, r2;

    r1 = pmQuatCartMult(&p1->rot, v2, vout);
    r2 = pmCartCartAdd(&p1->tran, vout, vout);

    return pmErrno = (r1 || r2) ? PM_NORM_ERR : PM_OK;
}

int pmCartAbs(PmCartesian const * const v, PmCartesian * const vout)
{

    vout->x = fabs(v->x);
    vout->y = fabs(v->y);
    vout->z = fabs(v->z);

    return pmErrno = PM_OK;
}

int pmCartNegEq(PmCartesian * const v1)
{
    v1->x = -v1->x;
    v1->y = -v1->y;
    v1->z = -v1->z;

    return pmErrno = PM_OK;
}

int pmCartCartSubEq(PmCartesian * const v, PmCartesian const * const v_sub)
{
    v->x -= v_sub->x;
    v->y -= v_sub->y;
    v->z -= v_sub->z;

    return pmErrno = PM_OK;
}

int pmMatCartMult(PmRotationMatrix const * const m, PmCartesian const * const v, PmCartesian * const vout)
{
    vout->x = m->x.x * v->x + m->y.x * v->y + m->z.x * v->z;
    vout->y = m->x.y * v->x + m->y.y * v->y + m->z.y * v->z;
    vout->z = m->x.z * v->x + m->y.z * v->y + m->z.z * v->z;

    return pmErrno = PM_OK;
}

int pmHomInv(PmHomogeneous const * const h1, PmHomogeneous * const h2)
{
    int r1, r2;

    r1 = pmMatInv(&h1->rot, &h2->rot);
    r2 = pmMatCartMult(&h2->rot, &h1->tran, &h2->tran);

    h2->tran.x *= -1.0;
    h2->tran.y *= -1.0;
    h2->tran.z *= -1.0;

    return pmErrno = (r1 || r2) ? PM_NORM_ERR : PM_OK;
}