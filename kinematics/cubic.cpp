//
// Created by Administrator on 2025/8/16.
//

#include "cubic.h"
#include "rtapi/rtapi_math.h"
#include "libnml/posemath.h"
#include "rtapi/rtapi.h"

#define SEGMENT_TIME_SET 0x01
#define INTERPOLATION_RATE_SET 0x02
#define ALL_SET (SEGMENT_TIME_SET | INTERPOLATION_RATE_SET)


static CUBIC_COEFF cubicCoeff(double x0, double v0,
			      double xn, double vn, double deltaT)
{
    CUBIC_COEFF retval;

    /* first the easy ones */
    retval.d = x0;
    retval.c = v0;

    /* now the hard ones */
    retval.b = 3 * (xn - x0) / (deltaT * deltaT) - (2 * v0 + vn) / deltaT;
    retval.a = (vn - v0) / (3.0 * (deltaT * deltaT)) - (2.0 * retval.b) / (3.0 * deltaT);

    return retval;
}

/*
   Interpolate points along a cubic, given t and cubic params
*/
static double interpolateCubic(CUBIC_COEFF coeff, double t)
{
    return coeff.a * (t * t * t) + coeff.b * (t * t) + coeff.c * t + coeff.d;
}

/*
   Interpolate velocity given same cubic params as above, using
   differentiation of cubic coeffs
*/
static double interpolateVel(CUBIC_COEFF coeff, double t)
{
    return 3.0 * coeff.a * (t * t) + 2.0 * coeff.b * t + coeff.c;
}

/*
   Interpolate acceleration given same cubic params as above, using
   differentiation twice of cubic coeffs
*/
static double interpolateAccel(CUBIC_COEFF coeff, double t)
{
    return 6.0 * coeff.a * t + 2.0 * coeff.b;
}

/*
   Interpolate jerk given same cubic params as above, using
   triple differentiation of cubic coeffs
*/
static double interpolateJerk(CUBIC_COEFF coeff, double t)
{
    (void)t;
    return 6.0 * coeff.a;
}

/*
   Calculate the cubic spline way point, given a point and its
   previous and successive neighbors
*/
static double wayPoint(double xMinus1, double x, double xPlus1)
{
    return (xMinus1 + 4.0 * x + xPlus1) / 6.0;
}

/*
   Calculate the cubic spline velocity value, given a point and its
   previous and successive neighbors
*/
static double velPoint(double xMinus1, double xPlus1, double deltaT)
{
    if (deltaT <= 0.0) {
	return 0.0;
    } else {
	return (xPlus1 - xMinus1) / (2.0 * deltaT);
    }
}

int cubicInit(CUBIC_STRUCT * ci)
{
    if (0 == ci) {
	return -1;
    }

    ci->configured = 0;
    ci->segmentTime = 0.0;
    ci->interpolationRate = 0;
    ci->interpolationIncrement = 0.0;
    cubicDrain(ci);

    return 0;
}

int cubicSetSegmentTime(CUBIC_STRUCT * ci, double time)
{
    if (0 == ci || time <= 0.0) {
	return -1;
    }

    ci->segmentTime = time;
    ci->configured |= SEGMENT_TIME_SET;
    if (ci->configured == ALL_SET) {
	ci->interpolationIncrement = ci->segmentTime / ci->interpolationRate;
    }

    return 0;
}

double cubicGetSegmentTime(CUBIC_STRUCT * ci)
{
    if (0 == ci || !(ci->configured & SEGMENT_TIME_SET)) {
	return 0.0;
    }

    return ci->segmentTime;
}

int cubicSetInterpolationRate(CUBIC_STRUCT * ci, int rate)
{
    if (0 == ci || rate <= 0) {
	return -1;
    }

    ci->interpolationRate = rate;
    ci->configured |= INTERPOLATION_RATE_SET;
    if (ci->configured == ALL_SET) {
	ci->interpolationIncrement = ci->segmentTime / ci->interpolationRate;
    }

    return 0;
}

int cubicGetInterpolationRate(CUBIC_STRUCT * ci)
{
    if (0 == ci || !(ci->configured & INTERPOLATION_RATE_SET)) {
	return 0;
    }

    return ci->interpolationRate;
}

double cubicGetInterpolationIncrement(CUBIC_STRUCT * ci)
{
    if (0 == ci || ci->configured != ALL_SET) {
	return 0;
    }

    return ci->interpolationIncrement;
}

CUBIC_COEFF cubicGetCubicCoeff(CUBIC_STRUCT * ci)
{
    CUBIC_COEFF errorReturn;

    if (0 == ci || !ci->filled) {
	errorReturn.a = 0.0;
	errorReturn.b = 0.0;
	errorReturn.c = 0.0;
	errorReturn.d = 0.0;

	return errorReturn;
    }

    return ci->coeff;
}

/*
  cubicAddPoint(double point)

  Add a point to the end of the cubic interpolator.

  Can only be called to initially fill up the four-point queue required
  for interpolation, or when interpolate() has been called for the full
  segment and the needNextPoint() flag is non-zero.

  The first point added fills the first two slots, since interpolation
  is done between the second and third input point, and this filling
  is required so that the output interpolation matches with the input
  points.
*/
int cubicAddPoint(CUBIC_STRUCT * ci, double point)
{
    if (0 == ci || !(ci->configured == ALL_SET)) {
	return -1;
    }

    if (!ci->needNextPoint) {
	return -1;
    }

    if (!ci->filled)
    {
	ci->x0 = point;
	ci->x1 = point;
	ci->x2 = point;
	ci->x3 = point;
	ci->filled = 1;
    } else {
	ci->x0 = ci->x1;
	ci->x1 = ci->x2;
	ci->x2 = ci->x3;
	ci->x3 = point;
    }

    /* calculate way points and coeff */
    ci->wp0 = wayPoint(ci->x0, ci->x1, ci->x2);
    ci->wp1 = wayPoint(ci->x1, ci->x2, ci->x3);
    ci->velp0 = velPoint(ci->x0, ci->x2, ci->segmentTime);
    ci->velp1 = velPoint(ci->x1, ci->x3, ci->segmentTime);
    ci->coeff = cubicCoeff(ci->wp0, ci->velp0, ci->wp1,ci->velp1, ci->segmentTime);
    ci->interpolationTime = 0.0;
    ci->needNextPoint = 0;

    return 0;
}

/*
    cubicOffset(CUBIC_STRUCT * ci, double offset)
   Set the interpolator so that the points inside are offset by the given
   value, as if the original points all had this offset added to them prior
   to their addition to the interpolator. This is used when offsetting
   trajectory points, to keep the interpolators consistent without draining
   them.
*/
int cubicOffset(CUBIC_STRUCT * ci, double offset)
{
    if (0 == ci || !(ci->configured == ALL_SET)) {
	return -1;
    }

    ci->x0 += offset;
    ci->x1 += offset;
    ci->x2 += offset;
    ci->x3 += offset;
    ci->wp0 += offset;
    ci->wp1 += offset;

    /* leave velp0, velp1 alone, since these are velocity points and are
       unaffected by position offsets */

    /* only the D coeff is affected, so we can change this directly */
    ci->coeff.d += offset;

    return 0;
}

int cubicFilled(CUBIC_STRUCT * ci)
{
    if (0 == ci) {
	return 0;
    }

    return ci->filled;
}

double cubicInterpolate(CUBIC_STRUCT * ci, double *x, double *v, double *a, double *j)
{
    double retval;

    if (0 == ci || !(ci->configured == ALL_SET)) {
	return 0.0;
    }

    if (ci->needNextPoint) {
	/* queue ran out-- fill right with last point */
	cubicAddPoint(ci, ci->x3);
    }

    retval = interpolateCubic(ci->coeff, ci->interpolationTime);

    /* do optional ones */
    if (x != 0) {
	*x = retval;
    }
    if (v != 0) {
	*v = interpolateVel(ci->coeff, ci->interpolationTime);
    }
    if (a != 0) {
	*a = interpolateAccel(ci->coeff, ci->interpolationTime);
    }
    if (j != 0) {
	*j = interpolateJerk(ci->coeff, ci->interpolationTime);
    }

    ci->interpolationTime += ci->interpolationIncrement;

    // rtapi_print_msg(RTAPI_MSG_DBG, "ci->segmentTime %f", ci->segmentTime);
    // rtapi_print_msg(RTAPI_MSG_DBG, " ci->interpolationTime %f", ci->interpolationTime);
    // rtapi_print_msg(RTAPI_MSG_DBG, " ci->interpolationIncrement %f\n", ci->interpolationIncrement);

    /* check to see if the next point is at (close to) the segment end */
    if (fabs(ci->segmentTime - ci->interpolationTime) < 0.5 * ci->interpolationIncrement)
    {
		/* just computed last point-- flag that we need a new one */
		ci->needNextPoint = 1;
    }

    return retval;
}

int cubicNeedNextPoint(CUBIC_STRUCT * ci)
{
    return ci->needNextPoint;
}

int cubicDrain(CUBIC_STRUCT * ci)
{
    ci->x0 = ci->x1 = ci->x2 = ci->x3 = 0.0;
    ci->wp0 = ci->wp1 = 0.0;
    ci->velp0 = ci->velp1 = 0.0;
    ci->filled = 0;
    ci->needNextPoint = 1;
    ci->coeff.a = 0.0;
    ci->coeff.b = 0.0;
    ci->coeff.c = 0.0;
    ci->coeff.d = 0.0;

    return 0;
}

