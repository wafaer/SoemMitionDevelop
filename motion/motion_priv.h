//
// Created by Administrator on 2025/8/16.
//

#ifndef MOTION_PRIV_H
#define MOTION_PRIV_H

#include "hal/hal.h"
#include "motion/motion.h"

extern emcmot_config_t *emcmotConfig;

typedef struct
{
    hal_s32_t *coarse_pos_cmd;/* RPI: commanded position, w/o comp */
    hal_s32_t *axis_vel_cmd;	/* RPI: commanded velocity, w/o comp */
    hal_s32_t *axis_acc_cmd;	/* RPI: commanded acceleration, w/o comp */
    hal_s32_t *axis_dec_cmd;	/* RPI: commanded acceleration, w/o comp */
    hal_s32_t *motor_offset;	/* RPI: motor offset, for checking homing stability */
    hal_s32_t *motor_pos_cmd;	/* WPI: commanded position, with comp */
    hal_s32_t *motor_pos_fb;	/* RPI: position feedback, with comp */
    hal_s32_t *axis_pos_cmd;	/* WPI: commanded position w/o comp, not ofs */
    hal_s32_t *axis_pos_fb;	/* RPI: position feedback, w/o comp */
    hal_s32_t *f_error;	/* RPI: following error */
    hal_s32_t *f_error_lim;	/* RPI: following error limit */
    hal_s32_t *axis_motion;
    hal_s32_t *axis_status;

    hal_bit_t *active;		/* RPI: joint is active, whatever that means */
    hal_bit_t *in_position;	/* RPI: joint is in position */
    hal_bit_t *error;		/* RPI: joint has an error */
    hal_bit_t *phl;		/* RPI: joint is at positive hard limit */
    hal_bit_t *nhl;		/* RPI: joint is at negative hard limit */
    hal_bit_t *f_errored;	/* RPI: joint had too much following error */
    hal_bit_t *faulted;		/* RPI: joint amp faulted */
    hal_bit_t *pos_lim_sw;	/* RPI: positive limit switch input */
    hal_bit_t *neg_lim_sw;	/* RPI: negative limit switch input */
    hal_bit_t *amp_fault;	/* RPI: amp fault input */ //伺服驱动故障
    hal_bit_t *amp_enable;	/* WPI: amp enable output */
    hal_bit_t *unlock;          /* WPI: command that axis should unlock for rotation */
    hal_bit_t *is_unlocked;     /* RPI: axis is currently unlocked */
    hal_bit_t *motionenable;
    hal_bit_t *motionrunning;

} axis_hal_t;

typedef struct {
    hal_bit_t *enable;		/* RPI: motion inhibit input */

    hal_bit_t *jog_inhibit;	/* RPI: set TRUE to inhibit jogging*/ /*禁止点动*/
    hal_bit_t *jog_stop;	/* RPI: set TRUE to stop jogging following accel values*/
    hal_bit_t *jog_stop_immediate;	/* RPI: set TRUE to stop jogging immediately*/
    hal_bit_t *jog_is_active;	/* RPI: TRUE if active jogging*/
    hal_bit_t *tp_reverse;	/* Set true if trajectory planner is running in reverse*/
    hal_bit_t *motion_enabled;	/* RPI: motion enable for all joints */
    hal_bit_t *in_position;	/* RPI: all joints are in position */
    hal_bit_t *coord_mode;	/* RPA: TRUE if coord, FALSE if free */

    hal_bit_t *coord_error;	/* RPA: TRUE if coord mode error */
    hal_bit_t *on_soft_limit;	/* RPA: TRUE if outside a limit */

    hal_bit_t *misc_error[EMCMOT_MAX_MISC_ERROR]; /* RPI array: output pins for misc error Inputs */
    // realtime overrun detection
    hal_u32_t   *last_period;	/* pin: last period in clocks */
    hal_float_t *last_period_ns;	/* pin: last period in nanoseconds */

    axis_hal_t axis[EMCMOT_MAX_AXIS];	/* data for each axis */

    hal_bit_t   *eoffset_active; /* ext offsets active */
    hal_bit_t   *eoffset_limited; /* ext offsets exceed limit */
} emcmot_hal_data_t;


#define ALL_AXES emcmotConfig->numAxes

#define GET_MOTION_ERROR_FLAG() (emcmotStatus->motionFlag & EMCMOT_MOTION_ERROR_BIT ? 1 : 0)

#define SET_MOTION_ERROR_FLAG(fl) if (fl) emcmotStatus->motionFlag |= EMCMOT_MOTION_ERROR_BIT; else emcmotStatus->motionFlag &= ~EMCMOT_MOTION_ERROR_BIT;

#define SET_MOTION_COORD_FLAG(fl) if (fl) emcmotStatus->motionFlag |= EMCMOT_MOTION_COORD_BIT; else emcmotStatus->motionFlag &= ~EMCMOT_MOTION_COORD_BIT;

#define SET_MOTION_FREE_FLAG(fl) if (fl) emcmotStatus->motionFlag |= EMCMOT_MOTION_FREE_BIT; else emcmotStatus->motionFlag &= ~EMCMOT_MOTION_FREE_BIT;

#define SET_MOTION_INPOS_FLAG(fl) if (fl) emcmotStatus->motionFlag |= EMCMOT_MOTION_INPOS_BIT; else emcmotStatus->motionFlag &= ~EMCMOT_MOTION_INPOS_BIT;

#define SET_MOTION_ENABLE_FLAG(fl) if (fl) emcmotStatus->motionFlag |= EMCMOT_MOTION_ENABLE_BIT; else emcmotStatus->motionFlag &= ~EMCMOT_MOTION_ENABLE_BIT;

#define SET_AXIS_INPOS_FLAG(axis,fl) if (fl) (axis)->flag |= EMCMOT_AXIS_INPOS_BIT; else (axis)->flag &= ~EMCMOT_AXIS_INPOS_BIT;

#define SET_AXIS_FERROR_FLAG(axis,fl) if (fl) (axis)->flag |= EMCMOT_AXIS_FERROR_BIT; else (axis)->flag &= ~EMCMOT_AXIS_FERROR_BIT;

#define SET_AXIS_PHL_FLAG(axis,fl) if (fl) (axis)->flag |= EMCMOT_AXIS_MAX_HARD_LIMIT_BIT; else (axis)->flag &= ~EMCMOT_AXIS_MAX_HARD_LIMIT_BIT;

#define SET_AXIS_NHL_FLAG(axis,fl) if (fl) (axis)->flag |= EMCMOT_AXIS_MIN_HARD_LIMIT_BIT; else (axis)->flag &= ~EMCMOT_AXIS_MIN_HARD_LIMIT_BIT;

#define GET_AXIS_PHL_FLAG(axis) ((axis)->flag & EMCMOT_AXIS_MAX_HARD_LIMIT_BIT ? 1 : 0)

#define GET_AXIS_NHL_FLAG(axis) ((axis)->flag & EMCMOT_AXIS_MIN_HARD_LIMIT_BIT ? 1 : 0)

#define SET_AXIS_FAULT_FLAG(axis,fl) if (fl) (axis)->flag |= EMCMOT_AXIS_FAULT_BIT; else (axis)->flag &= ~EMCMOT_AXIS_FAULT_BIT;

#define GET_MOTION_ENABLE_FLAG() (emcmotStatus->motionFlag & EMCMOT_MOTION_ENABLE_BIT ? 1 : 0)

#define GET_AXIS_ENABLE_FLAG(axis) ((axis)->flag & EMCMOT_AXIS_ENABLE_BIT ? 1 : 0)

#define GET_AXIS_ERROR_FLAG(axis) ((axis)->flag & EMCMOT_AXIS_ERROR_BIT ? 1 : 0)

#define SET_AXIS_ERROR_FLAG(axis,fl) if (fl) (axis)->flag |= EMCMOT_AXIS_ERROR_BIT; else (axis)->flag &= ~EMCMOT_AXIS_ERROR_BIT

#define GET_AXIS_FAULT_FLAG(axis) ((axis)->flag & EMCMOT_AXIS_FAULT_BIT ? 1 : 0)

#define GET_AXIS_FERROR_FLAG(axis) ((axis)->flag & EMCMOT_AXIS_FERROR_BIT ? 1 : 0)

#define GET_MOTION_INPOS_FLAG() (emcmotStatus->motionFlag & EMCMOT_MOTION_INPOS_BIT ? 1 : 0)

#define SET_AXIS_ENABLE_FLAG(axis,fl) if (fl) (axis)->flag |= EMCMOT_AXIS_ENABLE_BIT; else (axis)->flag &= ~EMCMOT_AXIS_ENABLE_BIT;

#define GET_MOTION_COORD_FLAG() (emcmotStatus->motionFlag & EMCMOT_MOTION_COORD_BIT ? 1 : 0)

#define GET_MOTION_FREE_FLAG() (emcmotStatus->motionFlag & EMCMOT_MOTION_FREE_BIT ? 1 : 0)

#define GET_AXIS_ACTIVE_FLAG(axis) ((axis)->flag & EMCMOT_AXIS_ACTIVE_BIT ? 1 : 0)

#define SET_AXIS_ACTIVE_FLAG(axis,fl) if (fl) (axis)->flag |= EMCMOT_AXIS_ACTIVE_BIT; else (axis)->flag &= ~EMCMOT_AXIS_ACTIVE_BIT;

#define GET_AXIS_INPOS_FLAG(axis) ((axis)->flag & EMCMOT_AXIS_INPOS_BIT ? 1 : 0)


extern void emcmotController(void *arg, long period);
extern void emcmotCommandHandler(void *arg, long period);
extern void refresh_jog_limits(emcmot_axis_t *joint,int axis_num);;
extern void emcmotSetRotaryUnlock(int axis, int unlock);
extern int emcmotGetRotaryIsUnlocked(int axis);

#endif //MOTION_PRIV_H
