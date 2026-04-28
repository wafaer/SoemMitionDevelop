//
// Created by Administrator on 2025/8/14.
//

#include "motion.h"
#include "rtapi/rtapi.h"
#include <float.h>
#include <stdatomic.h>
#include "axis.h"
#include "tp/tp.h"
#include "hal/hal.h"

#include "motion_struct.h"
#include "motion_priv.h"

extern int num_axis;
emcmot_struct_t *emcmotStruct = 0;
emcmot_command_t *emcmotCommand = 0;
emcmot_status_t *emcmotStatus = 0;
emcmot_config_t *emcmotConfig = 0;
emcmot_internal_t *emcmotInternal = 0;
emcmot_error_t *emcmotError = 0;
emcmot_hal_data_t *emcmot_hal_data = 0;

extern atomic_int_fast32_t atomic_actpos;
extern atomic_int_fast32_t atomic_targetpos;

static int emc_shmem_id;
static int mot_comp_id;
static int servo_comp_id;
static int uart_comp_id;
static int key = DEFAULT_SHMEM_KEY;
emcmot_axis_t axes[EMCMOT_MAX_AXIS];
static long base_period_nsec = 1000000;
static long servo_period_nsec = 1000000;
static long traj_period_nsec = 0;
int base_thread_fp = 0;

static int module_intfc();
static int init_hal_param(void);
static int export_axis(int num, axis_hal_t * addr);

static int module_intfc() {

	tpMotFunctions(emcmotSetRotaryUnlock,emcmotGetRotaryIsUnlocked,axis_get_vel_limit,axis_get_acc_limit);

	tpMotData(emcmotStatus,emcmotConfig);
	return 0;
}

static int tp_init()
{
    if (-1 == tpCreate(&emcmotInternal->coord_tp, DEFAULT_TC_QUEUE_SIZE,mot_comp_id))
    {
        rtapi_print_msg(RTAPI_MSG_ERR,"MOTION: tpCreate failed\n");
        return -1;
    }

    // tpInit is called from tpCreate
    tpSetCycleTime(&emcmotInternal->coord_tp,  emcmotConfig->trajCycleTime);
    tpSetVmax(     &emcmotInternal->coord_tp,  emcmotStatus->Maxvel, emcmotStatus->Maxvel);
    tpSetAmax(     &emcmotInternal->coord_tp,  emcmotStatus->Maxacc);
    tpSetPos(      &emcmotInternal->coord_tp, &emcmotStatus->carte_pos_cmd);
    return 0;
}

int motion_thread_main(void)
{
    int retval;

	retval = emcTrajInit();
	if (retval != 0) {
		rtapi_print_msg(RTAPI_MSG_ERR, "MOTION: emcTrajInit failed\n");
		hal_exit(mot_comp_id);
		return -1;
	}

    /* connect to the HAL and RTAPI */
    mot_comp_id = hal_init("motmod");
    if (mot_comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "MOTION: hal_init failed\n");
	return -1;
    }
    if (( num_axis < 1 ) || ( num_axis > EMCMOT_MAX_AXIS )) {
	rtapi_print_msg(RTAPI_MSG_ERR,"MOTION: num_joints is %d, must be between 1 and %d\n", num_axis, EMCMOT_MAX_AXIS);
	hal_exit(mot_comp_id);
	return -1;
    }

	/* initialize/export HAL pins and parameters */
	retval = init_hal_param();
	if (retval != 0) {
		rtapi_print_msg(RTAPI_MSG_ERR, "MOTION: init_hal_io failed\n");
		hal_exit(mot_comp_id);
		return -1;
	}

    /* allocate/initialize user space comm buffers (cmd/status/err) */
    retval = init_motion_comm_buffers();
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, ("MOTION: init_comm_buffers failed\n"));
	hal_exit(mot_comp_id);
	return -1;
    }

	//
	module_intfc();

	//轨迹规划
    if (tp_init())
    {
	rtapi_print_msg(RTAPI_MSG_ERR, ("MOTION: tp_init failed\n"));
	return -1;
    }

    /* set up for realtime execution of code */
    retval = init_motion_threads();
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, ("MOTION: init_threads failed\n"));
	hal_exit(mot_comp_id);
	return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "MOTION: rtapi_app_main complete\n");

    hal_ready(mot_comp_id);

    return 0;
}

void motion_thread_exit(void)
{
	int retval;

	retval = hal_stop_threads();
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
			("MOTION: hal_stop_threads() failed, returned %d\n"), retval);
	}

	/* free shared memory */
	retval = rtapi_shmem_delete(emc_shmem_id, mot_comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
			("MOTION: rtapi_shmem_delete() failed, returned %d\n"), retval);
	}
	/* disconnect from HAL and RTAPI */
	retval = hal_exit(mot_comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
			("MOTION: hal_exit() failed, returned %d\n"), retval);
	}
}

static int init_motion_comm_buffers(void)
{
    int axis_num, n;
    emcmot_axis_t *axis;
    int retval;

    emcmotStruct = 0;
    emcmotInternal = 0;
    emcmotStatus = 0;
    emcmotCommand = 0;
    emcmotConfig = 0;

    /* allocate and initialize the shared memory structure */
    emc_shmem_id = rtapi_shmem_new(key, mot_comp_id, sizeof(emcmot_struct_t));
    if (emc_shmem_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MOTION: rtapi_shmem_new failed, returned %d\n", emc_shmem_id);
	return -1;
    }
    retval = rtapi_shmem_getptr(emc_shmem_id, (void **) &emcmotStruct);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MOTION: rtapi_shmem_getptr failed, returned %d\n", retval);
	return -1;
    }

    /* we'll reference emcmotStruct directly */
    emcmotCommand = &emcmotStruct->command;
    emcmotStatus = &emcmotStruct->status;
    emcmotConfig = &emcmotStruct->config;
    emcmotInternal = &emcmotStruct->internal;
    emcmotError = &emcmotStruct->error;

    /* init error struct */
    emcmotErrorInit(emcmotError);

    /* init status struct */
    emcmotStatus->head = 0;
    emcmotStatus->commandEcho = 0;
    emcmotStatus->commandNumEcho = 0;
    emcmotStatus->commandStatus = 0;

    /* init more stuff */
    emcmotInternal->head = 0;
    emcmotConfig->head = 0;
	emcmotInternal->enabling = 0;
    emcmotStatus->motionFlag = 0;

    SET_MOTION_ERROR_FLAG(0);
    SET_MOTION_COORD_FLAG(0);

    emcmotInternal->split = 0;
    emcmotStatus->heartbeat = 0;

    ZERO_EMC_POSE(emcmotStatus->carte_pos_cmd);
    ZERO_EMC_POSE(emcmotStatus->carte_pos_fb);
    emcmotStatus->Maxvel = 0;
    emcmotConfig->limitVel = 0;
    emcmotStatus->Maxacc = 0;
    emcmotStatus->feed_scale = 1.0;
    emcmotStatus->rapid_scale = 1.0;
    emcmotStatus->net_feed_scale = 1.0;

	emcmotConfig->arcBlendOptDepth = 50;
	emcmotConfig->arcBlendRampFreq = 100;

    emcmotStatus->enables_new = FS_ENABLED | SS_ENABLED | FH_ENABLED;
    emcmotStatus->enables_queued = emcmotStatus->enables_new;
    emcmotStatus->id = 0;
    emcmotStatus->depth = 0;
    emcmotStatus->activeDepth = 0;
    emcmotStatus->paused = 0;
    emcmotStatus->overrideLimitMask = 0;
    SET_MOTION_INPOS_FLAG(1);
    SET_MOTION_ENABLE_FLAG(0);
    emcmot_config_change();

    axis_init_all();

    /* init per-joint stuff */
    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++)
    {
		/* point to structure for this joint */
		axis = &axes[axis_num];

		/* init the config fields with some "reasonable" defaults" */
		axis->type = 0;
		axis->max_pos_limit = 1;
		axis->min_pos_limit = -1;
		axis->vel_limit = 1;
		axis->acc_limit = 1;
		axis->min_ferror = 400;
		axis->max_ferror = 27486951;

		/* init joint flags */
		axis->flag = 1;
		SET_AXIS_INPOS_FLAG(axis, 1);

		/* init status info */
		axis->coarse_pos = 0;
		axis->pos_cmd = 0;
		axis->vel_cmd = 0;
		axis->acc_cmd = 0;
		axis->motor_pos_cmd = 0;
		axis->motor_pos_fb = 0;
		axis->pos_fb = 0;
		axis->ferror = 0;
		axis->ferror_limit = axis->min_ferror;
		axis->ferror_high_mark = 0;

		cubicInit(&(axis->cubic));
    }

    emcmotStatus->tail = 0;
	emcmotConfig->tail = emcmotConfig->head;

    return 0;
}

void updata_axis_param(int numAxes)
{
	for (int i = 0; i < numAxes; i++)
	{
		emcmot_axis_t *axis = &axes[i];
		axis->free_tp.curr_pos = atomic_load(&atomic_actpos);
	}
}

void emcmot_config_change(void)
{
	if (emcmotConfig->head == emcmotConfig->tail) {
		emcmotConfig->config_num++;
		emcmotStatus->config_num = emcmotConfig->config_num;
		emcmotConfig->head++;
	}
}

static int init_motion_threads(void)
{
    int retval;

	if (traj_period_nsec == 0) {
		traj_period_nsec = servo_period_nsec*1000;
	}

    retval = hal_create_thread("motion-thread", servo_period_nsec*1000, 1,98);
    if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,"MOTION: failed to create %ld nsec motion thread\n", servo_period_nsec);
		return -1;
    }

	retval = hal_export_funct("motion-command-handler", emcmotCommandHandler, 0	/* arg
	 */ , 1 /* uses_fp */ , 0 /* reentrant */ , mot_comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
			"MOTION: failed to export command handler function\n");
		return -1;
	}

    /* export realtime functions that do the real work */
    retval = hal_export_funct("motion-controller", emcmotController, 0	/* arg
	 */ , 1 /* uses_fp */ , 0 /* reentrant */ , mot_comp_id);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MOTION: failed to export controller function\n");
	return -1;
    }

	hal_add_funct_to_thread("motion-command-handler", "motion-thread", 1);
	hal_add_funct_to_thread("motion-controller", "motion-thread", 2);

    // if we don't set cycle times based on these guesses, emc doesn't
    // start up right
    setServoCycleTime(servo_period_nsec * 1e-9);
    setTrajCycleTime(traj_period_nsec * 1e-9);

    return 0;
}

void emcmotSetCycleTime(unsigned long nsec )
{
    int servo_mult;
    servo_mult = traj_period_nsec / nsec;
    if(servo_mult < 0) servo_mult = 1;
    setTrajCycleTime(nsec * 1e-9);
    setServoCycleTime(nsec * servo_mult * 1e-9);
}

/* call this when setting the trajectory cycle time */
static int setTrajCycleTime(double secs)
{
    static int t;

    /* make sure it's not zero */
    if (secs <= 0.0) {
	return -1;
    }

    emcmot_config_change();

    /* compute the interpolation rate as nearest integer to traj/servo */
    if(emcmotConfig->servoCycleTime)
        emcmotConfig->interpolationRate = (int) (secs / emcmotConfig->servoCycleTime + 0.5);
    else
        emcmotConfig->interpolationRate = 1;

    /* set traj planner */
    tpSetCycleTime(&emcmotInternal->coord_tp, secs);

    /* set the free planners, cubic interpolation rate and segment time */
    for (t = 0; t < EMCMOT_MAX_AXIS; t++) {
	cubicSetInterpolationRate(&(axes[t].cubic), emcmotConfig->interpolationRate);
    }

    /* copy into status out */
    emcmotConfig->trajCycleTime = secs;
	emcmotConfig->tail = emcmotConfig->head;

    return 0;
}

/* call this when setting the servo cycle time */
static int setServoCycleTime(double secs)
{
    static int t;

    /* make sure it's not zero */
    if (secs <= 0.0) {
	return -1;
    }

    emcmot_config_change();

    /* compute the interpolation rate as nearest integer to traj/servo */
    emcmotConfig->interpolationRate =
	(int) (emcmotConfig->trajCycleTime / secs + 0.5);

    /* set the cubic interpolation rate and PID cycle time */
    for (t = 0; t < EMCMOT_MAX_AXIS; t++)
    {
		cubicSetInterpolationRate(&(axes[t].cubic), emcmotConfig->interpolationRate);
    	cubicSetSegmentTime(&(axes[t].cubic), emcmotConfig->trajCycleTime);
    }

    /* copy into status out */
    emcmotConfig->servoCycleTime = secs;
	emcmotConfig->tail = emcmotConfig->head;

    return 0;
}

static int export_axis(int num, axis_hal_t * addr)
{
	int retval;

	if((retval = hal_pin_newf(HAL_S32, &(addr->coarse_pos_cmd), mot_comp_id, "axis.%d.coarse-pos-cmd",num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->axis_vel_cmd), mot_comp_id, "axis.%d.vel-cmd",num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->axis_acc_cmd), mot_comp_id, "axis.%d.acc-cmd", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->axis_dec_cmd), mot_comp_id, "axis.%d.dec-cmd", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->motor_pos_cmd), mot_comp_id, "axis.%d.motor-pos-cmd", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->motor_pos_fb), mot_comp_id, "axis.%d.motor-pos-fb", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->motor_offset), mot_comp_id, "axis.%d.motor-offset", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->axis_pos_cmd), mot_comp_id, "axis.%d.pos-cmd", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->axis_pos_fb), mot_comp_id, "axis.%d.pos-fb", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->f_error), mot_comp_id, "axis.%d.f-error", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->f_error_lim), mot_comp_id, "axis.%d.f-error-lim", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->axis_motion), mot_comp_id, "axis.%d.axis_motion", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_S32, &(addr->axis_status), mot_comp_id, "axis.%d.axis_status", num)) !=0) return retval;


	if((retval = hal_pin_newf(HAL_BIT, &(addr->active), mot_comp_id, "axis.%d.active", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->motionenable), mot_comp_id, "axis.%d.motionenable", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->motionrunning), mot_comp_id, "axis.%d.motionrunning", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->in_position), mot_comp_id, "axis.%d.in-position", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->error), mot_comp_id, "axis.%d.error", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->phl), mot_comp_id, "axis.%d.pos-hard-limit", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->nhl), mot_comp_id, "axis.%d.neg-hard-limit", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->f_errored), mot_comp_id, "axis.%d.f-errored", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->faulted), mot_comp_id, "axis.%d.faulted", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->pos_lim_sw), mot_comp_id, "axis.%d.pos-lim-sw-in", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->neg_lim_sw), mot_comp_id, "axis.%d.neg-lim-sw-in", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->amp_fault), mot_comp_id, "axis.%d.amp-fault-in", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->amp_enable), mot_comp_id, "axis.%d.amp-enable-out", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->unlock), mot_comp_id, "axis.%d.unlock", num)) !=0) return retval;
	if((retval = hal_pin_newf(HAL_BIT, &(addr->is_unlocked), mot_comp_id, "axis.%d.is-unlocked", num)) !=0) return retval;

	return 0;
}

static int init_hal_param(void)
{
	int n, retval;
	axis_hal_t      *axis_data;
	emcmot_hal_data = hal_malloc(sizeof(emcmot_hal_data_t));
	if (emcmot_hal_data == 0)
	{
		rtapi_print_msg(RTAPI_MSG_ERR, "MOTION: emcmot_hal_data malloc failed\n");
		return -1;
	}

	hal_pin_newf(HAL_BIT, &(emcmot_hal_data->jog_inhibit), mot_comp_id,"motion.jog-inhibit");
	hal_pin_newf(HAL_BIT, &(emcmot_hal_data->jog_stop), mot_comp_id, "motion.jog-stop");
	hal_pin_newf(HAL_BIT,  &(emcmot_hal_data->jog_stop_immediate), mot_comp_id, "motion.jog-stop-immediate");
	hal_pin_newf(HAL_BIT,  &(emcmot_hal_data->enable), mot_comp_id, "motion.enable");
	hal_pin_newf(HAL_BIT,  &(emcmot_hal_data->motion_enabled), mot_comp_id, "motion.motion-enabled");
	hal_pin_newf(HAL_BIT,  &(emcmot_hal_data->in_position), mot_comp_id, "motion.in-position");
	hal_pin_newf(HAL_BIT,  &(emcmot_hal_data->coord_mode), mot_comp_id, "motion.coord-mode");
	hal_pin_newf(HAL_BIT,  &(emcmot_hal_data->coord_error), mot_comp_id, "motion.coord-error");
	hal_pin_newf(HAL_BIT,  &(emcmot_hal_data->on_soft_limit), mot_comp_id, "motion.on-soft-limit");
	hal_pin_newf(HAL_U32,  &(emcmot_hal_data->last_period), mot_comp_id, "motion.servo.last-period");
	hal_pin_newf(HAL_U32,  &(emcmot_hal_data->jog_is_active), mot_comp_id, "motion.jog-is-active");
	hal_pin_newf(HAL_BIT,  &(emcmot_hal_data->tp_reverse), mot_comp_id, "motion.tp-reverse");
	hal_pin_newf(HAL_FLOAT, &(emcmot_hal_data->last_period_ns), mot_comp_id, "motion.servo.last-period-ns");
	hal_pin_newf(HAL_BIT,  &(emcmot_hal_data->eoffset_limited), mot_comp_id, "motion.eoffset-limited");
	hal_pin_newf(HAL_BIT,  &(emcmot_hal_data->eoffset_active), mot_comp_id, "motion.eoffset-active");


	/* initialize machine parameters */
	*(emcmot_hal_data->jog_inhibit) = 0;
	*(emcmot_hal_data->jog_stop) = 0;
	*(emcmot_hal_data->jog_stop_immediate) = 0;
	*(emcmot_hal_data->enable) = 1;
	*(emcmot_hal_data->motion_enabled) = 0;
	*(emcmot_hal_data->in_position) = 0;
	*(emcmot_hal_data->coord_mode) = 0;
	*(emcmot_hal_data->coord_error) = 0;
	*(emcmot_hal_data->on_soft_limit) = 0;
	*(emcmot_hal_data->last_period) = 0;

	for (n = 0; n < num_axis; n++)
	{
		axis_data = &(emcmot_hal_data->axis[n]);
		/* export all vars */
		retval = export_axis(n, axis_data);
		if (retval != 0) {
			rtapi_print_msg(RTAPI_MSG_ERR, "MOTION: axis %d pin/param export failed\n", n);
			return -1;
		}
		*(axis_data->amp_enable) = 1;
	}

	axis_init_hal_param(mot_comp_id);

	return 0;
}

