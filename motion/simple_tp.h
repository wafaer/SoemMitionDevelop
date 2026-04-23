//
// Created by Administrator on 2025/8/16.
//

#ifndef SIMPLE_TP_H
#define SIMPLE_TP_H
#include <stdint.h>

#define TINY_DP(max_acc,period) (max_acc*period*period*0.001)

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct simple_tp_t
    {
        int32_t pos_cmd;		/* position command */
        int32_t max_vel;		/* velocity limit */
        int32_t max_acc;		/* acceleration limit */
        int32_t vel;		    /* velocity */
        int32_t acc;		    /* acceleration*/
        int enable;		    /* if zero, motion stops ASAP */
        int32_t curr_pos;	/* current position */
        int32_t curr_vel;	/* current velocity */
        int active;		/* non-zero if motion in progress */
        int dir;
    } simple_tp_t;

    extern void simple_tp_update(simple_tp_t *tp, int32_t period);


#ifdef __cplusplus
}
#endif
#endif //SIMPLE_TP_H
