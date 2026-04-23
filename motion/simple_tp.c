//
// Created by Administrator on 2025/8/16.
//

#include "simple_tp.h"
#include "rtapi/rtapi_math.h"
#include "rtapi/rtapi.h"

void simple_tp_update(simple_tp_t *tp, int32_t period)
{
    double max_dv, tiny_dp, pos_err, vel_req;

    tp->active = 0;
    /* compute max change in velocity per servo period */
    max_dv = tp->acc * period;
    /* compute a tiny position range, to be treated as zero */
    tiny_dp = TINY_DP(tp->acc, period);
    /* calculate desired velocity */
    if (tp->enable) {
        /* planner enabled, request a velocity that tends to drive
           pos_err to zero, but allows for stopping without position
           overshoot */
        pos_err = tp->pos_cmd - tp->curr_pos;
        // rtapi_print_msg(RTAPI_MSG_DBG, "pos_err %f\n", pos_err);
        /* positive and negative errors require some sign flipping to avoid sqrt(negative) */
        if (pos_err > tiny_dp)
        {
            vel_req = -max_dv + sqrt(2 * tp->acc * pos_err + max_dv * max_dv);
            /* mark planner as active */
            tp->active = 1;
        } else if (pos_err < -tiny_dp)
        {
            vel_req =  max_dv - sqrt(-2 * tp->acc * pos_err + max_dv * max_dv);
            /* mark planner as active */
            tp->active = 1;
        } else {
            /* within 'tiny_dp' of desired pos, no need to move */
            vel_req = 0;
            tp->enable = 0;
        }

    } else {
        /* planner disabled, request zero velocity */
        vel_req = 0;
        /* and set command to present position to avoid movement when
           next enabled */
        tp->pos_cmd = tp->curr_pos;
    }
    /* limit velocity request */
    if (vel_req > tp->vel) {
        vel_req = tp->vel;
    } else if (vel_req < -tp->vel) {
        vel_req = -tp->vel;
    }
    /* ramp velocity toward request at accel limit */
    if (vel_req > tp->curr_vel + max_dv) {
        tp->curr_vel += max_dv;
    } else if (vel_req < tp->curr_vel - max_dv) {
        tp->curr_vel -= max_dv;
    } else {
        tp->curr_vel = vel_req;
    }

    // rtapi_print_msg(RTAPI_MSG_DBG, "tp->curr_vel %d\n", tp->curr_vel);

    /* check for still moving */
    if (tp->curr_vel != 0) {
        /* yes, mark planner active */
        tp->active = 1;
    }

    tp->curr_pos += tp->curr_vel * period;
}