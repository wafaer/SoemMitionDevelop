//
// Created by Administrator on 2025/8/16.
//

#include "motion.h"
#include "emcmotcfg.h"
#include "motion_priv.h"
#include "simple_tp.h"
#include "motion/axis.h"
#include "hal/hal.h"
#include "hal/ServoEnable.h"
#include <stdatomic.h>
#include "rtapi/rtapi.h"
#include "tp/tp.h"
#include "rtapi/rtapi_math.h"

extern emcmot_status_t *emcmotStatus;
extern emcmot_hal_data_t *emcmot_hal_data;
extern emcmot_internal_t *emcmotInternal;
extern emcmot_config_t *emcmotConfig;
extern emcmot_axis_t axes[EMCMOT_MAX_AXIS];

extern  ecat_status_t *emcecatStatus;
static double *pcmd_p[EMCMOT_MAX_AXIS];
static int32_t servo_period;
static unsigned long last_period = 0;
static bool   coord_cubic_active = 0;
static int    ext_offset_coord_limit  = 0;
static int timest = 0;

extern atomic_int_fast32_t atomic_targetpos[EMCMOT_MAX_AXIS];
extern atomic_int_fast32_t atomic_actpos[EMCMOT_MAX_AXIS];

static void handle_interpolation(void);
static void process_inputs(void);
static void check_for_faults(void);
static void set_operating_mode(void);
static void get_pos_cmds(long period);
static void output_to_hal(void);
static void update_status(void);

void emcmotController(void *arg, long period)
{

    (void)arg;
    static int do_once = 1;
	emcmot_axis_t *axis;
    if (do_once) {
        pcmd_p[0] = &(emcmotStatus->carte_pos_cmd.tran.x);
        pcmd_p[1] = &(emcmotStatus->carte_pos_cmd.tran.y);
        pcmd_p[2] = &(emcmotStatus->carte_pos_cmd.tran.z);
        // pcmd_p[3] = &(emcmotStatus->carte_pos_cmd.a);
        // pcmd_p[4] = &(emcmotStatus->carte_pos_cmd.b);
        // pcmd_p[5] = &(emcmotStatus->carte_pos_cmd.c);
        // pcmd_p[6] = &(emcmotStatus->carte_pos_cmd.u);
        // pcmd_p[7] = &(emcmotStatus->carte_pos_cmd.v);
        // pcmd_p[8] = &(emcmotStatus->carte_pos_cmd.w);
        do_once = 0;
    }

    static long long int last = 0;
	axis = &axes[0];
    long long int now = rtapi_get_clocks();
    long int this_run = (long int)(now - last);
    *(emcmot_hal_data->last_period) = this_run;
#ifdef HAVE_CPU_KHZ
    *(emcmot_hal_data->last_period_ns) = this_run * 1e6 / cpu_khz;
#endif

    // we need this for next time
    last = now;

    /* calculate servo period as a double - period is in integer nsec */
    servo_period = period * 0.000000001;
	if(servo_period < 1)
	{
		servo_period = 1;
	}

    if(period != (long)last_period) {
        emcmotSetCycleTime(period);
        last_period = period;
    }

    /* increment head count to indicate work in progress */
    emcmotStatus->head++;
    /* here begins the core of the controller */

	handle_interpolation();

	//处理轴数据，位置反馈、误差计算和限位检测
    process_inputs();
	//检查故障
    check_for_faults();

	//设置运动模式
	set_operating_mode();

    get_pos_cmds(period);

    output_to_hal();

    update_status();
    /* here ends the core of the controller */
    emcmotStatus->heartbeat++;
    /* set tail to head, to indicate work complete */
    emcmotStatus->tail = emcmotStatus->head;
/* end of controller function */
}

static void handle_interpolation(void)
{

	emcmotStatus->carte_pos_cmd.tran.x = axes[0].pos_cmd;
	emcmotStatus->carte_pos_cmd.tran.y = axes[1].pos_cmd;
	emcmotStatus->carte_pos_cmd.tran.z = axes[2].pos_cmd;

	tpSetPos(&emcmotInternal->coord_tp, &emcmotStatus->carte_pos_cmd);
}

static void process_inputs(void)
{
    int axis_num;
    int32_t abs_ferror, scale;
    axis_hal_t *axis_data;
    emcmot_axis_t *axis;
    unsigned char enables;

	//根据运动状态选择使能信号的来源
    /* compute net feed and spindle scale factors */
    if ( emcmotStatus->motion_state == EMCMOT_MOTION_COORD ) {
	/* use the enables that were queued with the current move */
	enables = emcmotStatus->enables_queued;
    } else {
	/* use the enables that are in effect right now */
	enables = emcmotStatus->enables_new;
    }

    scale = 1;
    if (   (emcmotStatus->motion_state != EMCMOT_MOTION_FREE)
        && (enables & FS_ENABLED) ) {
        if (emcmotStatus->motionType == 1) {
            scale *= emcmotStatus->rapid_scale;
        } else
        {
            scale *= emcmotStatus->feed_scale;
        }
    }

	//读取并处理每个轴的输入
    /* read and process per-joint inputs */
    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS ; axis_num++)
    {
		/* point to axis HAL data */
		axis_data = &(emcmot_hal_data->axis[axis_num]);
		/* point to axis data */
		axis = &axes[axis_num];
		if (!GET_AXIS_ACTIVE_FLAG(axis)) {
		    /* if joint is not active, skip it */
		    continue;
		}

		/* copy data from HAL to joint structure */
		axis->motor_pos_fb = *(axis_data->motor_pos_fb);
    	// rtapi_print_msg(RTAPI_MSG_DBG, "axis->motor_pos_fb %d\n", axis->motor_pos_fb);
		// if(timest % 500 == 0)
		// {
		// 	rtapi_print_msg(RTAPI_MSG_DBG, "axis->motor_pos_fb %f\n", axis->motor_pos_fb);
		// }
    	//计算反馈位置
	    axis->pos_fb = axis->motor_pos_fb - axis->motor_offset;
    	// rtapi_print_msg(RTAPI_MSG_DBG, "axis->pos_fb %d\n", axis->pos_fb);
		// if(timest % 500 == 0)
		// {
		// 	rtapi_print_msg(RTAPI_MSG_DBG, "axis->pos_fb %f\n", axis->pos_fb);
		// }
    	
    	//计算系统跟随误差
		/* calculate following error */
	    axis->ferror = axis->pos_cmd - axis->pos_fb;
		abs_ferror = (int32_t)fabs(axis->ferror);
    	// rtapi_print_msg(RTAPI_MSG_DBG, "abs_ferror %d\n", abs_ferror);
		// if(timest % 500 == 0)
		// {
		// 	rtapi_print_msg(RTAPI_MSG_DBG, "abs_ferror %f\n", abs_ferror);
		// }

    	// 更新最大跟随误差
		/* update maximum ferror if needed */
		// if (abs_ferror > axis->ferror_high_mark) {
		//     axis->ferror_high_mark = abs_ferror;
		// }

    	// 计算跟随误差限制
		/* calculate following error limit */
		// if(timest % 500 == 0)
		// {
		// 	rtapi_print_msg(RTAPI_MSG_DBG, "axis->vel_cmd %f\n", axis->vel_cmd);
		// }
    	// rtapi_print_msg(RTAPI_MSG_DBG, "axisnum %d vel_cmd %d\n", axis_num, axis->vel_cmd);
		if (axis->vel_limit > 0) {
			int64_t temp = (int64_t)axis->max_ferror * axis->vel_cmd / axis->vel_limit;
			// rtapi_print_msg(RTAPI_MSG_DBG, "temp %ld", temp);

		    axis->ferror_limit = temp;
			// rtapi_print_msg(RTAPI_MSG_DBG, " axis->ferror_limit %d\n", axis->ferror_limit);

		} else {
		    axis->ferror_limit = 0;
		}
		if (axis->ferror_limit < axis->min_ferror) {
		    axis->ferror_limit = axis->min_ferror;
		}
    	// rtapi_print_msg(RTAPI_MSG_DBG, "axisnum %d  vel_cmd %d abs_ferror %d axis->ferror_limit %d\n", axis_num, axis->vel_cmd, abs_ferror, axis->ferror_limit);
		// if(timest % 500 == 0)
		// {
		// 	rtapi_print_msg(RTAPI_MSG_DBG, "axis->ferror_limit %f\n", axis->ferror_limit);
		// }
    	// 更新跟随错误标志
		/* update following error flag */
		if (abs_ferror > axis->ferror_limit)
		{
		    SET_AXIS_FERROR_FLAG(axis, 1);
		} else {
		    SET_AXIS_FERROR_FLAG(axis, 0);
		}

    	// 读取正负限位限位开关
		/* read limit switches */
		if (*(axis_data->pos_lim_sw)) {
		    SET_AXIS_PHL_FLAG(axis, 1);
		} else {
		    SET_AXIS_PHL_FLAG(axis, 0);
		}
		if (*(axis_data->neg_lim_sw)) {
		    SET_AXIS_NHL_FLAG(axis, 1);
		} else {
		    SET_AXIS_NHL_FLAG(axis, 0);
		}
		axis->on_pos_limit = GET_AXIS_PHL_FLAG(axis);
		axis->on_neg_limit = GET_AXIS_NHL_FLAG(axis);

    	// 读取伺服驱动故障输入
    	if (*(axis_data->amp_fault)) {
    		SET_AXIS_FAULT_FLAG(axis, 1);  // 故障
    	} else {
    		SET_AXIS_FAULT_FLAG(axis, 0);  // 正常
    	}

    }
	// 如果点动进行中且收到停止请求，则停止点动
    // if jog in progress stop the jog if requested 点动
    if (enables & *(emcmot_hal_data->jog_is_active) && (*(emcmot_hal_data->jog_stop) || *(emcmot_hal_data->jog_stop_immediate)))
    {
        axis_jog_abort_all(*(emcmot_hal_data->jog_stop_immediate));
    }

	timest++;
}

static void check_for_faults(void)
{
    int axis_num, error_num;
    emcmot_axis_t *axis;
    int neg_limit_override, pos_limit_override;

	//检查全局使能
    /* check for various global fault conditions */
    /* only check enable input if running */

    if ( GET_MOTION_ENABLE_FLAG() != 0 ) {
	if ( *(emcmot_hal_data->enable) == 0 ) {
	    emcmotInternal->enabling = 0;
	}
    }

	//检查轴故障
    /* check for various axis fault conditions */
    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++) {
	/* point to joint data */
	axis = &axes[axis_num];
	/* only check active, enabled axes */
	if ( GET_AXIS_ACTIVE_FLAG(axis) && GET_AXIS_ENABLE_FLAG(axis) ) {
	    /* are any limits for this joint overridden? */
		//轴限位是否被覆盖
	    neg_limit_override = emcmotStatus->overrideLimitMask & ( 1 << (axis_num*2));
	    pos_limit_override = emcmotStatus->overrideLimitMask & ( 2 << (axis_num*2));

	    if (pos_limit_override || neg_limit_override )
	    {
	    	if (!GET_AXIS_ERROR_FLAG(axis)) {
	    		/* report the error just this once */
	    		rtapi_print_msg(RTAPI_MSG_DBG, "joint %d on limit switch error\n", axis_num);
	    	}
	    	SET_AXIS_ERROR_FLAG(axis, 1);
	    	emcmotInternal->enabling = 0;
	    }
		//伺服驱动故障检查
	    /* check for amp fault */
	    if (GET_AXIS_FAULT_FLAG(axis)) {
			/* joint is faulted, trip */
			if (!GET_AXIS_ERROR_FLAG(axis)) {
			    /* report the error just this once */
			    rtapi_print_msg(RTAPI_MSG_DBG, "joint %d amplifier fault", axis_num);
			}
			SET_AXIS_ERROR_FLAG(axis, 1);
			emcmotInternal->enabling = 0;
	    }

		//跟随误差检查

	    /* check for excessive following error */
	    if (GET_AXIS_FERROR_FLAG(axis)) {
			if (!GET_AXIS_ERROR_FLAG(axis))
			{
			    /* report the error just this once */
			    rtapi_print_msg(RTAPI_MSG_DBG, "axis %d following error\n", axis_num);
				print_msg(RTAPI_MSG_DBG, "axis %d following error\n", axis_num);
			}
			SET_AXIS_ERROR_FLAG(axis, 1);
			emcmotInternal->enabling = 0;
	    }
	/* end of if JOINT_ACTIVE_FLAG(joint) */
	}
    /* end of check for joint faults loop */
    }

    /* Check Miscellaneous faults */
    for (error_num=0; error_num < emcmotConfig->numMiscError; error_num++){
      if(emcmotStatus->misc_error[error_num] && GET_MOTION_ENABLE_FLAG()) {
        rtapi_print_msg(RTAPI_MSG_DBG, "Motion Stopped by misc error %d\n", error_num);
        emcmotInternal->enabling = 0;
      }
    }
}

static void set_operating_mode(void)
{
    int axis_num;
    emcmot_axis_t *axis;

    /* check for disabling */
	//motion disable 使能
    if (!emcmotInternal->enabling && GET_MOTION_ENABLE_FLAG())
    {
    	rtapi_print_msg(RTAPI_MSG_DBG, "axis disenable\n");
		//清空轨迹
		tpClear(&emcmotInternal->coord_tp);
		for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++)
		{
		    /* point to joint data */
		    axis = &axes[axis_num];
		    /* disable free mode planner */
			//去使能
		    axis->free_tp.enable = 0;
		    axis->free_tp.curr_vel = 0.0;
			//清空插补数据
		    /* drain coord mode interpolators */
		    cubicDrain(&(axis->cubic));
		    if (GET_AXIS_ACTIVE_FLAG(axis))
		    {
				SET_AXIS_INPOS_FLAG(axis, 1);
				SET_AXIS_ENABLE_FLAG(axis, 0);
		    }
		}

    	//中止所有点动运动
	    axis_jog_abort_all(1);
		SET_MOTION_ENABLE_FLAG(0);
    }

    //motion 使能
    if (emcmotInternal->enabling && !GET_MOTION_ENABLE_FLAG())
    {
    	rtapi_print_msg(RTAPI_MSG_DBG, "axis enable\n");
        if (*(emcmot_hal_data->eoffset_limited))
        {
            *(emcmot_hal_data->eoffset_limited) = 0;
        }
        axis_initialize_external_offsets();
        tpSetPos(&emcmotInternal->coord_tp, &emcmotStatus->carte_pos_cmd);
		for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++)
		{
		    /* point to joint data */
		    axis = &axes[axis_num];
		    axis->free_tp.curr_pos = axis->pos_cmd;
		    if (GET_AXIS_ACTIVE_FLAG(axis))
		    {
				SET_AXIS_ENABLE_FLAG(axis, 1);
		    }
		    /* clear any outstanding joint errors when going into enabled
		       state */
		    SET_AXIS_ERROR_FLAG(axis, 0);
		}

		SET_MOTION_ENABLE_FLAG(1);
		/* clear any outstanding motion errors when going into enabled state */
		SET_MOTION_ERROR_FLAG(0);
	}

	//插补模式
	//set coord cmd but current is not coord mode
	if (emcmotInternal->coordinating && !GET_MOTION_COORD_FLAG())
	{
		if (GET_MOTION_INPOS_FLAG()) {
			/* preset traj planner to current position */
			rtapi_print_msg(RTAPI_MSG_DBG, "set coord\n");
			// subtract at coord mode start
			axis_apply_ext_offsets_to_carte_pos(-1, pcmd_p);

			tpSetPos(&emcmotInternal->coord_tp, &emcmotStatus->carte_pos_cmd);
			/* drain the cubics so they'll synch up */
			for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++) {
				/* point to joint data */
				axis = &axes[axis_num];
				cubicDrain(&(axis->cubic));
				// rtapi_print_msg(RTAPI_MSG_DBG, "axis->cubic %d\n", axis->cubic.needNextPoint);
			}
			/* clear the override limits flags */
			emcmotInternal->overriding = 0;
			emcmotStatus->overrideLimitMask = 0;
			SET_MOTION_COORD_FLAG(1);
			SET_MOTION_ERROR_FLAG(0);
		} else {
			/* not in position-- don't honor mode change */
			emcmotInternal->coordinating = 0;
		}
	}

	//单轴运动
	//set free cmd but current is not free mode
	if (emcmotInternal->freeing && !GET_MOTION_FREE_FLAG())
	{
		if (GET_MOTION_INPOS_FLAG())
		{
			rtapi_print_msg(RTAPI_MSG_DBG, "set free\n");
			for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++)
			{
				/* point to joint data */
				axis = &axes[axis_num];
				/* set joint planner curr_pos to current location */
				axis->free_tp.curr_pos = axis->pos_cmd;
				/* but it can stay disabled until a move is required */
				axis->free_tp.enable = 0;
			}
			SET_MOTION_FREE_FLAG(1);
			SET_MOTION_ERROR_FLAG(0);
		} else {
			/* not in position-- don't honor mode change */
			emcmotInternal->freeing = 0;
		}
	}

    if (!GET_MOTION_ENABLE_FLAG())
    {
		emcmotStatus->motion_state = EMCMOT_MOTION_DISABLED;
    } else if (GET_MOTION_COORD_FLAG())
    {
		emcmotStatus->motion_state = EMCMOT_MOTION_COORD;
    } else {
		emcmotStatus->motion_state = EMCMOT_MOTION_FREE;
    }

} //set_operating_mode

static void get_pos_cmds(long period)
{
    int axis_num, result;
    emcmot_axis_t *axis;// 轴结构体指针
    int32_t positions[EMCMOT_MAX_AXIS]; // 位置数组
    int32_t vel_lim; // 速度限制

    /* used in teleop mode to compute the max accell requested */
    int onlimit = 0;
    int joint_limit[EMCMOT_MAX_AXIS][2];

    /* copy joint position feedback to local array */
  //   for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++)
  //   {
		// /* point to joint struct */
		// axis = &axes[axis_num];
		// /* copy coarse command */
	 //    //粗位置命令
		// positions[axis_num] = emcmotInternal->coord_tp.currentPos.tran.x;
  //   }

	positions[0] = (int32_t)emcmotInternal->coord_tp.currentPos.tran.x;
	positions[1] = (int32_t)emcmotInternal->coord_tp.currentPos.tran.y;
	positions[2] = (int32_t)emcmotInternal->coord_tp.currentPos.tran.z;

    /* RUN MOTION CALCULATIONS: */

    /* run traj planner code depending on the state */
    switch ( emcmotStatus->motion_state)
    {
		    case EMCMOT_MOTION_FREE:
				/* in free mode, each joint is planned independently */
				/* initial value for flag, if needed it will be cleared below */
				SET_MOTION_INPOS_FLAG(1);
				for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++)
				{
				    /* point to joint struct */
				    axis = &axes[axis_num];
				    if (!GET_AXIS_ACTIVE_FLAG(axis))
				    {
				        /* if joint is not active, skip it */
				        continue;
				    }

				    if(axis->acc_limit > emcmotStatus->Maxacc)
						axis->acc_limit = emcmotStatus->Maxacc;  // 加速度限制
				    /* compute joint velocity limit */
					if ( emcmotStatus->motion_state != EMCMOT_MOTION_FREE )
					{
						/* velocity limit = joint limit * global scale factor */
						/* the global factor is used for feedrate override */
						vel_lim = axis->vel_limit * emcmotStatus->net_feed_scale;
						/* must not be greater than the joint physical limit */
						if (vel_lim > axis->vel_limit) {
							vel_lim = axis->vel_limit;  // 速度限制
						}
						/* set vel limit in free TP */
						if (vel_lim < axis->free_tp.max_vel)
							axis->free_tp.max_vel = vel_lim;
					}

					/* set acc limit in free TP */
					axis->free_tp.max_acc = axis->acc_limit;

					//更新规划器
					// rtapi_print_msg(RTAPI_MSG_DBG, "FREE axis->free_tp.pos_cmd %d\n", axis->free_tp.pos_cmd);
					// rtapi_print_msg(RTAPI_MSG_DBG, "FREE axis->free_tp.curr_pos %d\n", axis->free_tp.curr_pos);

					// rtapi_print_msg(RTAPI_MSG_DBG, "aFREE xis->free_tp.enable %d\n", axis->free_tp.enable);

					simple_tp_update(&(axis->free_tp), servo_period );

					// rtapi_print_msg(RTAPI_MSG_DBG, "FREE axis->free_tp.curr_pos %d\n", axis->free_tp.curr_pos);
					atomic_store(&atomic_actpos[axis_num], axis->free_tp.curr_pos);
					// rtapi_print_msg(RTAPI_MSG_DBG, "atomic_actpos %f\n", atomic_actpos);

					axis->pos_cmd = axis->free_tp.curr_pos;  // 设置位置命令
					axis->vel_cmd = axis->free_tp.curr_vel;  // 设置速度命令
					axis->acc_cmd = 0;
					axis->coarse_pos = axis->free_tp.curr_pos; // 更新粗位置

					//如果规划器仍在工作设置不在位
					if ( axis->free_tp.active ) {
						/* active TP means we're moving, so not in position */
						SET_AXIS_INPOS_FLAG(axis, 0);
						SET_MOTION_INPOS_FLAG(0);
						/* is any limit disabled for this move? */
						if ( emcmotStatus->overrideLimitMask )
						{
				           emcmotInternal->overriding = 1;
						}
					}
					else
					{
						SET_AXIS_INPOS_FLAG(axis, 1);
					}
				}

				//如果关掉限位并且运动到位，开启限位
    			if ( (emcmotInternal->overriding ) && ( GET_MOTION_INPOS_FLAG() ) ) {
    				emcmotStatus->overrideLimitMask = 0;
    				emcmotInternal->overriding = 0;
    			}
    			/* end of FREE mode */
    			break;

    		case EMCMOT_MOTION_COORD:
    			//停止所有增量运动
    			axis_jog_abort_all(1);

    			//启动三次插补
    			coord_cubic_active = 1;
    			//当插补器需要下一个点时，运行协调轨迹规划周期
    			// rtapi_print_msg(RTAPI_MSG_DBG, "axes[0].cubic.needNextPoint %d\n", axes[0].cubic.needNextPoint);
    			while (cubicNeedNextPoint(&(axes[0].cubic)))
    			{
    				tpRunCycle(&emcmotInternal->coord_tp, period);
    				// 获取笛卡尔位置
    				tpGetPos(&emcmotInternal->coord_tp, &emcmotStatus->carte_pos_cmd);

    				positions[0] = (int32_t)emcmotStatus->carte_pos_cmd.tran.x;
    				positions[1] = (int32_t)emcmotStatus->carte_pos_cmd.tran.y;
    				positions[2] = (int32_t)emcmotStatus->carte_pos_cmd.tran.z;

					//更新坐标并检查边界
    				if (axis_update_coord_with_bound(pcmd_p, servo_period)) {
    					ext_offset_coord_limit = 1;
    				} else {
    					ext_offset_coord_limit = 0;
    				}

    				for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++)
    				{
    					axis = &axes[axis_num];
    					if (!GET_AXIS_ACTIVE_FLAG(axis)) continue;

    					axis = &axes[axis_num];
    					// 设置粗位置
    					axis->coarse_pos = positions[axis_num];
    					// 添加点到三次插补器
    					cubicAddPoint(&(axis->cubic), axis->coarse_pos);
    				}
				}

				for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS;  axis_num++)
				{
					axis = &axes[axis_num];
					if (!GET_AXIS_ACTIVE_FLAG(axis)) continue;

					// rtapi_print_msg(RTAPI_MSG_DBG, " axis->cubic.coeff.a %f", axis->cubic.coeff.a);
					// rtapi_print_msg(RTAPI_MSG_DBG, " axis->cubic.coeff.b %f", axis->cubic.coeff.b);
					// rtapi_print_msg(RTAPI_MSG_DBG, " axis->cubic.coeff.c %f", axis->cubic.coeff.c);
					// rtapi_print_msg(RTAPI_MSG_DBG, " axis->cubic.coeff.d %f\n", axis->cubic.coeff.d);

				    axis = &axes[axis_num];
					// 三次插补获取位置、速度和加速度
					double vel_cmd;
					double acc_cmd;
				    axis->pos_cmd = (int32_t)cubicInterpolate(&(axis->cubic), 0, &vel_cmd, &acc_cmd, 0);

					axis->vel_cmd = (int32_t)vel_cmd;
					axis->acc_cmd = (int32_t)acc_cmd;
					atomic_store(&atomic_actpos[axis_num], axis->pos_cmd);

					rtapi_print_msg(RTAPI_MSG_DBG, "axis %d pos_cmd %d", axis_num, axis->pos_cmd);
					rtapi_print_msg(RTAPI_MSG_DBG, " vel_cmd %d\n", axis->vel_cmd);
				}

				/* report motion status */
				SET_MOTION_INPOS_FLAG(0);
    			// 如果轨迹规划完成，设置到位标志
				if (tpIsDone(&emcmotInternal->coord_tp)) {
				    SET_MOTION_INPOS_FLAG(1);
				}
				break;

		    case EMCMOT_MOTION_DISABLED:
				//指令位置等于反馈位置
				emcmotStatus->carte_pos_cmd = emcmotStatus->carte_pos_fb;
				for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++)
				{
				    /* point to joint struct */
				    axis = &axes[axis_num];
				    /* save old command */
				    axis->pos_cmd = axis->pos_fb;
				    /* set joint velocity and acceleration to zero */
				    axis->vel_cmd = 0.0;
				    axis->acc_cmd = 0.0;
				}

				break;
		    default:
				break;
    }

	//检查软限位
    for (axis_num = 0; axis_num < ALL_AXES; axis_num++)
    {
		/* point to joint data */
		axis = &axes[axis_num];

		/* Zero values */
		joint_limit[axis_num][0] = 0;
		joint_limit[axis_num][1] = 0;

		/* check for soft limits */
		if (axis->pos_cmd > axis->max_pos_limit + 0.000000000001)
		{
		    joint_limit[axis_num][1] = 1;
	            onlimit = 1;
	    }
	    else if (axis->pos_cmd < axis->min_pos_limit - 0.000000000001)
	    {
		    joint_limit[axis_num][0] = 1;
	            onlimit = 1;
	    }
    }

    if ( onlimit ) //如果有软限位触发
    {
		if ( ! emcmotStatus->on_soft_limit )
		{
		    for (axis_num = 0; axis_num < emcmotConfig->numAxes; axis_num++) {
		        if (joint_limit[axis_num][0] == 1)
		        	{
	                    axis = &axes[axis_num];
	                    rtapi_print_msg(RTAPI_MSG_ERR, "超过负限位 (%.5d) 在轴 %d\n",
	                                  axis->min_pos_limit, axis_num);
	                } else if (joint_limit[axis_num][1] == 1) {
	                    axis = &axes[axis_num];
	                    rtapi_print_msg(RTAPI_MSG_ERR, "超过正限位 (%.5d) 在轴 %d\n",
	                                  axis->max_pos_limit,axis_num);
	                }
		    }
		    SET_MOTION_ERROR_FLAG(1);
		    emcmotStatus->on_soft_limit = 1;
		}
    } else {
	emcmotStatus->on_soft_limit = 0;
    }
} // get_pos_cmds()

//更新电机位置 status
static void output_to_hal(void)
{
	int axis_num;
	emcmot_axis_t *axis;
	axis_hal_t *axis_data;

	*(emcmot_hal_data->motion_enabled) = GET_MOTION_ENABLE_FLAG();
	*(emcmot_hal_data->in_position) = GET_MOTION_INPOS_FLAG();
	*(emcmot_hal_data->coord_mode) = GET_MOTION_COORD_FLAG();
	*(emcmot_hal_data->coord_error) = GET_MOTION_ERROR_FLAG();
	*(emcmot_hal_data->on_soft_limit) = emcmotStatus->on_soft_limit;

	for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++)
	{
		/* point to joint struct */
		axis = &axes[axis_num];
		axis_data = &(emcmot_hal_data->axis[axis_num]);

		if(!GET_AXIS_ACTIVE_FLAG(axis))
		{
			continue;
		}

		/* apply backlash and motor offset to output */
		//计算电机位置命令：将关节位置命令加上反向间隙滤波值和电机偏移量
		axis->motor_pos_cmd = axis->pos_cmd + axis->motor_offset;

		// rtapi_print_msg(RTAPI_MSG_DBG, "hal axis->motor_pos_cmd %d\n", axis->motor_pos_cmd);

		// //更新电机位置
		atomic_store(&atomic_targetpos[axis_num], axis->motor_pos_cmd);

		*(axis_data->motor_offset) = axis->motor_offset;            //电机偏移量
		*(axis_data->motor_pos_cmd) = axis->motor_pos_cmd;           //电机位置命令
		*(axis_data->axis_pos_cmd) = axis->pos_cmd;                 //原始轴位置命令
		*(axis_data->axis_pos_fb) = axis->pos_fb;                   //轴位置反馈
		*(axis_data->amp_enable) = GET_AXIS_ENABLE_FLAG(axis);      //使能状态
		*(axis_data->coarse_pos_cmd) = axis->coarse_pos;             //粗位置命令
		*(axis_data->axis_vel_cmd) = axis->vel_cmd;                 //轴速度
		*(axis_data->axis_acc_cmd) = axis->acc_cmd;                 //轴加速度
		*(axis_data->f_error) = axis->ferror;                        //跟随误差
		*(axis_data->f_error_lim) = axis->ferror_limit;              //跟随误差限制值

		*(axis_data->active) = GET_AXIS_ACTIVE_FLAG(axis);          //轴激活状态
		*(axis_data->motionenable) = emcmotInternal->enabling;
		*(axis_data->motionrunning) = axis->free_tp.enable;
		*(axis_data->in_position) = GET_AXIS_INPOS_FLAG(axis);      //轴在位状态（是否到达目标位置）
		*(axis_data->error) = GET_AXIS_ERROR_FLAG(axis);            //轴错误状态
		*(axis_data->phl) = GET_AXIS_PHL_FLAG(axis);                //正硬限位状态
		*(axis_data->nhl) = GET_AXIS_NHL_FLAG(axis);                //负硬限位状态
		*(axis_data->f_errored) = GET_AXIS_FERROR_FLAG(axis);       //跟随错误状态
		*(axis_data->faulted) = GET_AXIS_FAULT_FLAG(axis);          //输出故障状态

		// rtapi_print_msg(RTAPI_MSG_DBG, "hp axis_data->axis_pos_fb %d\n", (int32_t)*axis_data->axis_pos_fb);
    }

	axis_output_to_hal(pcmd_p);

	*(emcmot_hal_data->jog_is_active) = axis_jog_is_active();
}

static void update_status(void)
{
    int axis_num, misc_error;
    emcmot_axis_t *axis;
    emcmot_axis_status_t *axis_status;

    /* copy status info from private joint structure to status
       struct in shared memory */
    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++) {
	/* point to joint data */
	axis = &axes[axis_num];
	/* point to joint status */
	axis_status = &(emcmotStatus->axis_status[axis_num]);
	/* copy stuff */

	axis_status->flag = axis->flag;
	axis_status->pos_cmd = axis->pos_cmd;
	axis_status->pos_fb = axis->pos_fb;
	axis_status->vel_cmd = axis->vel_cmd;
	axis_status->acc_cmd = axis->acc_cmd;
	axis_status->ferror = axis->ferror;
	axis_status->ferror_high_mark = axis->ferror_high_mark;
	axis_status->max_pos_limit = axis->max_pos_limit;
	axis_status->min_pos_limit = axis->min_pos_limit;
	axis_status->min_ferror = axis->min_ferror;
	axis_status->max_ferror = axis->max_ferror;
    }


    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++) {
        /* point to axis status */
        axis_status = &(emcmotStatus->axis_status[axis_num]);

        axis_status->axis_vel_cmd = axis_get_teleop_vel_cmd(axis_num);
        axis_status->max_pos_limit = axis_get_max_pos_limit(axis_num);
        axis_status->min_pos_limit = axis_get_min_pos_limit(axis_num);
    }
    emcmotStatus->eoffset_pose.tran.x = axis_get_ext_offset_curr_pos(0);
    emcmotStatus->eoffset_pose.tran.y = axis_get_ext_offset_curr_pos(1);
    emcmotStatus->eoffset_pose.tran.z = axis_get_ext_offset_curr_pos(2);
    // emcmotStatus->eoffset_pose.a      = axis_get_ext_offset_curr_pos(3);
    // emcmotStatus->eoffset_pose.b      = axis_get_ext_offset_curr_pos(4);
    // emcmotStatus->eoffset_pose.c      = axis_get_ext_offset_curr_pos(5);
    // emcmotStatus->eoffset_pose.u      = axis_get_ext_offset_curr_pos(6);
    // emcmotStatus->eoffset_pose.v      = axis_get_ext_offset_curr_pos(7);
    // emcmotStatus->eoffset_pose.w      = axis_get_ext_offset_curr_pos(8);

    emcmotStatus->external_offsets_applied = *(emcmot_hal_data->eoffset_active);

    for (misc_error=0; misc_error < emcmotConfig->numMiscError; misc_error++){
      emcmotStatus->misc_error[misc_error] = *(emcmot_hal_data->misc_error[misc_error]);
    }

    emcmotStatus->jogging_active = *(emcmot_hal_data->jog_is_active);

    /*! \todo FIXME - the rest of this function is stuff that was apparently
       dropped in the initial move from emcmot.c to control.c.  I
       don't know how much is still needed, and how much is baggage.
    */

    /* motion emcmotInternal->coord_tp status */
    emcmotStatus->depth = tpQueueDepth(&emcmotInternal->coord_tp);
    emcmotStatus->activeDepth = tpActiveDepth(&emcmotInternal->coord_tp);
    emcmotStatus->id = tpGetExecId(&emcmotInternal->coord_tp);
    //KLUDGE add an API call for this
    emcmotStatus->reverse_run = emcmotInternal->coord_tp.reverse_run;

    emcmotStatus->motionType = tpGetMotionType(&emcmotInternal->coord_tp);
    emcmotStatus->queueFull = tcqFull(&emcmotInternal->coord_tp.queue);

    /* check to see if we should pause in order to implement
       single emcmotStatus->stepping */

    if (emcmotStatus->stepping && emcmotInternal->idForStep != emcmotStatus->id) {
      tpPause(&emcmotInternal->coord_tp);
      emcmotStatus->stepping = 0;
      emcmotStatus->paused = 1;
    }
}
