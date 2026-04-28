//
// Created by Administrator on 2025/8/15.
//

#include <osal.h>
#include <stdlib.h>
#include <time.h>
#include "axis.h"
#include "motion.h"
#include "motion_priv.h"
#include "motion_struct.h"
#include "rtapi/rtapi.h"
#include "rtapi/rtapi_mutex.h"
#include "rtapi/rtapi_math.h"
#include "tp/tp.h"

extern emcmot_struct_t *emcmotStruct;
extern emcmot_command_t *emcmotCommand;
extern emcmot_status_t *emcmotStatus;
extern emcmot_internal_t *emcmotInternal;
extern emcmot_axis_t axes[EMCMOT_MAX_AXIS];
extern emcmot_hal_data_t *emcmot_hal_data;

// static int time_val = 0;
// struct timespec now, next;
// static int total_val = 0;
// static int per_val = 1;
// static int first_flag = 0;

static int joint_jog_ok(int axis_num, double vel)
{
    emcmot_axis_t *axis;
    int neg_limit_override, pos_limit_override;

    /* point to joint data */
    axis = &axes[axis_num];
    /* are any limits for this joint overridden? */
    neg_limit_override = emcmotStatus->overrideLimitMask & ( 1 << (axis_num*2));
    pos_limit_override = emcmotStatus->overrideLimitMask & ( 2 << (axis_num*2));
    if ( neg_limit_override && pos_limit_override )
    {
		return 1;
    }
    if (axis_num < 0 || axis_num >= ALL_AXES)
    {
		return 0;
    }
    if (vel > 0.0 && GET_AXIS_PHL_FLAG(axis))
    {
		return 0;
    }
    if (vel < 0.0 && GET_AXIS_NHL_FLAG(axis)) {
		return 0;
    }
    // refresh_jog_limits(axis,axis_num);
    if ( vel > 0.0 && (axis->pos_cmd > axis->max_jog_limit) ) {
		return 0;
    }
    if ( vel < 0.0 && (axis->pos_cmd < axis->min_jog_limit) ) {
		return 0;
    }

    return 1;
}

void refresh_jog_limits(emcmot_axis_t *axis, int axis_num)
{
    int32_t range;

	range = axis->max_pos_limit - axis->min_pos_limit;
	axis->max_jog_limit = axis->pos_fb + range;
	axis->min_jog_limit = axis->pos_fb - range;
}

static int limits_ok(void)
{
	int joint_num;
	emcmot_axis_t *axis;

	for (joint_num = 0; joint_num < ALL_AXES; joint_num++)
	{
		axis = &axes[joint_num];
		if (!GET_AXIS_ACTIVE_FLAG(axis)) {
			continue;
		}

		if (GET_AXIS_PHL_FLAG(axis) || GET_AXIS_NHL_FLAG(axis)) {
			return 0;
		}
	}

	return 1;
}

void emcmotCommandHandler_locked(void *arg, long servo_period)
{
    (void)arg;
    int axis_num;
    emcmot_axis_t *axis;

    int32_t tmp1;
    char issue_atspeed = 0;

    if (emcmotCommand->commandNum != emcmotStatus->commandNumEcho)
    {
        emcmotStatus->head++;
        emcmotInternal->head++;

        emcmotStatus->commandEcho = emcmotCommand->command;
        emcmotStatus->commandNumEcho = emcmotCommand->commandNum;

        emcmotStatus->commandStatus = EMCMOT_COMMAND_OK;

        axis_num = emcmotCommand->axis;

    	axis = &axes[axis_num];

    	switch (emcmotCommand->command)
	    {
    		//配置参数
    		case EMCMOT_DOWNLOADS_CONFIG:
    			axis->acc_cmd = emcmotCommand->acc;
    			axis->vel_cmd = emcmotCommand->vel;
    			axis->acc_limit = emcmotCommand->Maxacc;
    			axis->vel_limit = emcmotCommand->Maxvel;
    			axis->max_pos_limit = emcmotCommand->maxLimit;
    			axis->min_pos_limit = emcmotCommand->minLimit;
    			axis->max_jog_limit = emcmotCommand->maxLimit;
    			axis->min_jog_limit = emcmotCommand->minLimit;
    			emcmotStatus->Maxacc = emcmotCommand->Maxacc;
    			emcmotStatus->Maxvel = emcmotCommand->Maxvel;
    			emcmotInternal->coord_tp.tolerance = 200;

    			emcmotConfig->maxAxisScale = emcmotCommand->maxAxisScale;
    			emcmotConfig->limitVel = emcmotCommand->Maxvel;
    			// refresh_jog_limits(axis,axis_num);

    			rtapi_print_msg(RTAPI_MSG_INFO, "axis %d DOWNLOADS_CONFIG\n",axis_num);
    			break;

    		//使能
    		case EMCMOT_AXIS_ENABLE:

    			rtapi_print_msg(RTAPI_MSG_DBG, "axis %d ENABLE\n",axis_num);
    			if ( *(emcmot_hal_data->enable) == 0 )
    			{
    				rtapi_print_msg(RTAPI_MSG_DBG,"axis %d can't enable motion, enable input is false\n",axis_num);
    			} else
    			{
    				*(emcmot_hal_data->axis[axis_num].amp_enable) = 1;
    				SET_AXIS_ACTIVE_FLAG(axis, 1);
    			}
    			break;

    			//去使能
    		case EMCMOT_AXIS_DISABLE:
    			rtapi_print_msg(RTAPI_MSG_DBG, "axis %d DISABLE\n",axis_num);
    			*(emcmot_hal_data->axis[axis_num].amp_enable) = 0;
    			SET_AXIS_ACTIVE_FLAG(axis, 0);
    			break;

    		case EMCMOT_MOTION_ENABLE:
    			rtapi_print_msg(RTAPI_MSG_DBG, "axis %d MOTION ENABLE\n",axis_num);
    			emcmotInternal->enabling = 1;
    			break;

    		case EMCMOT_MOTION_DISABLE:
    			rtapi_print_msg(RTAPI_MSG_DBG, "axis %d MOTION DISABLE\n",axis_num);
    			emcmotInternal->enabling = 0;
    			break;

    		//停止运动控制
	        case EMCMOT_ABORT:
	            rtapi_print_msg(RTAPI_MSG_DBG, "ABORT %d", axis_num);
    			/*停止规划器*/
	            if (GET_MOTION_COORD_FLAG())
	            {
	                tpAbort(&emcmotInternal->coord_tp);
	            } else
	            {
	                for (axis_num = 0; axis_num < ALL_AXES; axis_num++)
	                {
	                    axis = &axes[axis_num];
	                    axis->free_tp.enable = 0;
	                }
	            }
    			/*更新轴状态*/
	            SET_MOTION_ERROR_FLAG(0);
	            for (axis_num = 0; axis_num < ALL_AXES; axis_num++)
	            {
	                axis = &axes[axis_num];
	                SET_AXIS_ERROR_FLAG(axis, 0);
	                SET_AXIS_FAULT_FLAG(axis, 0);
	            }
	            emcmotStatus->paused = 0;
	            break;

    		//运动暂停
    		case EMCMOT_PAUSE:
    			rtapi_print_msg(RTAPI_MSG_DBG, "axis %d PAUSE\n",axis_num);
    			tpPause(&emcmotInternal->coord_tp);
    			emcmotStatus->paused = 1;
    			break;

    		// Continuous motion
    		case EMCMOT_JOG_CONT:
    			rtapi_print_msg(RTAPI_MSG_DBG, "JOG_CONT\n");
    			rtapi_print_msg(RTAPI_MSG_DBG, " %d", axis_num);
    			if (!GET_MOTION_ENABLE_FLAG()) {
    				rtapi_print_msg(RTAPI_MSG_DBG,("Can't jog joint when not enabled\n"));
    				SET_AXIS_ERROR_FLAG(axis, 1);
    				break;
    			}
    			/*检查禁止标志*/
    			if (*(emcmot_hal_data->jog_inhibit)){
    				rtapi_print_msg(RTAPI_MSG_DBG,"axis %d Cannot jog while jog-inhibit is active\n",axis_num);
    				break;
    			}

    			/*检查限位*/
    			if (!joint_jog_ok(axis_num, emcmotCommand->Maxvel)) {
    				SET_AXIS_ERROR_FLAG(axis, 1);
    				break;
    			}
    			/*设置点动目标位置*/
    			// refresh_jog_limits(axis,axis_num);
    			if (emcmotCommand->Maxvel > 0.0) {
    				axis->free_tp.pos_cmd = axis->max_jog_limit;
    			} else {
    				axis->free_tp.pos_cmd = axis->min_jog_limit;
    			}

    			/* set velocity of jog */
    			if (axis->vel_cmd < emcmotCommand->Maxvel)
    			{
    				axis->free_tp.vel = axis->vel_cmd;
    			}else
    			{
    				axis->free_tp.vel = emcmotCommand->Maxvel;
    			}
    			axis->free_tp.max_vel = emcmotCommand->Maxvel;
    			/* use max joint accel */
    			if (axis->acc_cmd < emcmotCommand->Maxacc)
    			{
    				axis->free_tp.acc = axis->acc_cmd;
    			}else
    			{
    				axis->free_tp.acc = emcmotCommand->Maxacc;
    			}
    			axis->free_tp.max_acc = axis->acc_limit;
    			/* and let it go */
    			axis->free_tp.enable = 1;

    			axis_jog_abort_all(0);
    			SET_AXIS_ERROR_FLAG(axis, 0);
    			break;

    		//单轴相对运动
	        case EMCMOT_JOG_INCR:
			    //相对增量点动
			    rtapi_print_msg(RTAPI_MSG_DBG, "JOG_INCR %d\n", axis_num);
			    if (!GET_MOTION_ENABLE_FLAG()) {
					rtapi_print_msg(RTAPI_MSG_DBG, "axis %d Can't jog joint when not enabled\n",axis_num);
					SET_AXIS_ERROR_FLAG(axis, 1);
					break;
			    }
    			/*如果禁止点动*/
		        if (*(emcmot_hal_data->jog_inhibit)){
		               rtapi_print_msg(RTAPI_MSG_DBG, "axis %d Cannot jog while jog-inhibit is active\n",axis_num);
		           break;
		        }
    			/*检查限位*/
				if (!joint_jog_ok(axis_num, emcmotCommand->Maxvel))
				{
					SET_AXIS_ERROR_FLAG(axis, 1);
					break;
				}
    		    //计算目标位置，正向/负向偏移
    			/* set target position for jog */
    			if (emcmotCommand->dir)
    			{
    				tmp1 = axis->free_tp.pos_cmd + emcmotCommand->offset;
    			} else {
    				tmp1 = axis->free_tp.pos_cmd - emcmotCommand->offset;
    			}
    		    // 刷新关节点动限位
    			/* don't jog past limits */
    			// refresh_jog_limits(axis,axis_num);
    			// 限位检查
    			if (tmp1 > axis->max_jog_limit) {
    				break;
    			}
    			if (tmp1 < axis->min_jog_limit) {
    				break;
    			}

    			// 执行点动
    			/* set target position */
    			axis->free_tp.pos_cmd = tmp1;
    		    rtapi_print_msg(RTAPI_MSG_DBG, "cmd axis->free_tp.pos_cmd %d\n", axis->free_tp.pos_cmd);
    			/* set velocity of jog */
    			if (axis->vel_cmd < emcmotCommand->Maxvel)
    			{
    				axis->free_tp.vel = axis->vel_cmd;
    			}else
    			{
    				axis->free_tp.vel = emcmotCommand->Maxvel;
    			}

    			axis->free_tp.max_vel = emcmotCommand->Maxvel;
    			/* use max joint accel */
    			if (axis->acc_cmd < emcmotCommand->Maxacc)
    			{
    				axis->free_tp.acc = axis->acc_cmd;
    			}else
    			{
    				axis->free_tp.acc = emcmotCommand->Maxacc;
    			}
    			axis->free_tp.acc = axis->acc_cmd;
    			axis->free_tp.max_acc = axis->acc_limit;
    			//set dir
    			axis->free_tp.dir = emcmotCommand->dir;
    			/* and let it go */
    			axis->free_tp.enable = 1;
    			axis_jog_abort(axis_num, 0);

				rtapi_print_msg(RTAPI_MSG_DBG, "axis->vel_cmd %d\n", axis->vel_cmd);
				
    			SET_AXIS_ERROR_FLAG(axis, 0);
			    break;

    		//单轴绝对
			case EMCMOT_JOG_ABS:
			    /* do an absolute jog */
			    //绝对运动
			    rtapi_print_msg(RTAPI_MSG_DBG, "JOG_ABS"" %d\n", axis_num);
			    if (!GET_MOTION_ENABLE_FLAG()) {
				rtapi_print_msg(RTAPI_MSG_DBG, "Can't jog joint when not enabled\n");
				SET_AXIS_ERROR_FLAG(axis, 1);
				break;
			    }
		        /*检查禁止标志*/
		        if (*(emcmot_hal_data->jog_inhibit)){
		               rtapi_print_msg(RTAPI_MSG_DBG,("Cannot jog while jog-inhibit is active\n"));
		            break;
		        }

    			/*检查限位*/
    			if (!joint_jog_ok(axis_num, emcmotCommand->Maxvel)) {
    				SET_AXIS_ERROR_FLAG(axis, 1);
    				break;
    			}

    			axis->free_tp.pos_cmd = emcmotCommand->offset;
    			// refresh_jog_limits(axis,axis_num);
    			if (axis->free_tp.pos_cmd > axis->max_jog_limit) {
    				axis->free_tp.pos_cmd = axis->max_jog_limit;
    			}
    			if (axis->free_tp.pos_cmd < axis->min_jog_limit) {
    				axis->free_tp.pos_cmd = axis->min_jog_limit;
    			}
    			/* set velocity of jog */
    			if (axis->vel_cmd < emcmotCommand->Maxvel)
    			{
    				axis->free_tp.vel = axis->vel_cmd;
    			}else
    			{
    				axis->free_tp.vel = emcmotCommand->Maxvel;
    			}
    			axis->free_tp.max_vel = fabs(emcmotCommand->Maxvel);
    			/* use max joint accel */
    			if (axis->acc_cmd < emcmotCommand->Maxacc)
    			{
    				axis->free_tp.acc = axis->acc_cmd;
    			}else
    			{
    				axis->free_tp.acc = emcmotCommand->Maxacc;
    			}
    			axis->free_tp.max_acc = axis->acc_limit;
    			/* and let it go */
    			axis->free_tp.enable = 1;
    			SET_AXIS_ERROR_FLAG(axis, 0);
		        break;

    		//轴停止
    		case EMCMOT_JOG_ABORT:
    			if (axis == 0) { break; }
    			axis->free_tp.enable = 0;
    			/* update status flags */
    			SET_AXIS_ERROR_FLAG(axis, 0);
    			break;

    		//直线插补
    		case EMCMOT_SET_LINE:
			    rtapi_print_msg(RTAPI_MSG_DBG, "SET_LINE\n");
    			//检查使能和运动模式标志
			    if (!GET_MOTION_COORD_FLAG() || !GET_MOTION_ENABLE_FLAG()) {
					rtapi_print_msg(RTAPI_MSG_DBG,("need to be enabled, in coord mode for linear move\n"));
					emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_COMMAND;
					SET_MOTION_ERROR_FLAG(1);
					break;
			    }/*检查是否超出限位*/
    			else if (!limits_ok()) {
					rtapi_print_msg(RTAPI_MSG_DBG,("can't do linear move with limits exceeded\n"));
					emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_PARAMS;
					tpAbort(&emcmotInternal->coord_tp);
					SET_MOTION_ERROR_FLAG(1);
					break;
			    }

			    /* append it to the emcmotInternal->coord_tp */
			    tpSetId(&emcmotInternal->coord_tp, emcmotCommand->id);
    			if (emcmotCommand->ref)
    			{
    				if (emcmotCommand->dir)
    				{
    					emcmotCommand->pos.tran.x = emcmotInternal->coord_tp.goalPos.tran.x + emcmotCommand->pos.tran.x;
    					emcmotCommand->pos.tran.y = emcmotInternal->coord_tp.goalPos.tran.y + emcmotCommand->pos.tran.y;
    					emcmotCommand->pos.tran.z = emcmotInternal->coord_tp.goalPos.tran.z + emcmotCommand->pos.tran.z;
    				}else
    				{
    					emcmotCommand->pos.tran.x = emcmotInternal->coord_tp.goalPos.tran.x - emcmotCommand->pos.tran.x;
    					emcmotCommand->pos.tran.y = emcmotInternal->coord_tp.goalPos.tran.y - emcmotCommand->pos.tran.y;
    					emcmotCommand->pos.tran.z = emcmotInternal->coord_tp.goalPos.tran.z - emcmotCommand->pos.tran.z;
    				}
    			}

    			tpSetVlimit(&emcmotInternal->coord_tp, emcmotConfig->limitVel);

			    int res_addline = tpAddLine(&emcmotInternal->coord_tp,
							emcmotCommand->pos,
							emcmotCommand->motion_type,
							emcmotCommand->vel,
							emcmotCommand->Maxvel,
							emcmotCommand->acc,
							emcmotStatus->enables_new,
							issue_atspeed,
							emcmotCommand->turn);
				/*直线添加失败*/
		        if (res_addline < 0) {
		            rtapi_print_msg(RTAPI_MSG_DBG,("can't add linear move at line %d, error code %d"),
		                    emcmotCommand->id, res_addline);
		            emcmotStatus->commandStatus = EMCMOT_COMMAND_BAD_EXEC;
		            tpAbort(&emcmotInternal->coord_tp);
		            SET_MOTION_ERROR_FLAG(1);
		            break;
		        }

    			tpSetSpindleSync(&emcmotInternal->coord_tp,emcmotCommand->spindle, emcmotCommand->spindlesync, 0);
    			for (int t = 0; t < EMCMOT_MAX_AXIS; t++) {
    				axes[t].cubic.needNextPoint = 1;
    			}

			    break;

    		//圆弧插补
			case EMCMOT_SET_CIRCLE:
			    rtapi_print_msg(RTAPI_MSG_DBG, "SET_CIRCLE\n");
			    if (!GET_MOTION_COORD_FLAG() || !GET_MOTION_ENABLE_FLAG()) {
					rtapi_print_msg(RTAPI_MSG_DBG,("need to be enabled, in coord mode for circular move\n"));
					emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_COMMAND;
					SET_MOTION_ERROR_FLAG(1);
					break;
			    }/*检查限位*/
    			else if (!limits_ok()) {
					rtapi_print_msg(RTAPI_MSG_DBG,("can't do circular move with limits exceeded\n"));
					emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_PARAMS;
					tpAbort(&emcmotInternal->coord_tp);
					SET_MOTION_ERROR_FLAG(1);
					break;
			    }

			    /* append it to the emcmotInternal->coord_tp */
			    tpSetId(&emcmotInternal->coord_tp, emcmotCommand->id);
			    int res_addcircle = tpAddCircle(&emcmotInternal->coord_tp, emcmotCommand->pos,
		                            emcmotCommand->center, emcmotCommand->normal,
		                            emcmotCommand->turn, emcmotCommand->motion_type,
		                            emcmotCommand->Maxvel, emcmotCommand->ini_maxvel,
		                            emcmotCommand->Maxacc, emcmotStatus->enables_new,
									issue_atspeed);
		        if (res_addcircle < 0)
		        {
		            rtapi_print_msg(RTAPI_MSG_DBG,("can't add circular move at line %d, error code %d\n"),
		                    emcmotCommand->id, res_addcircle);
					emcmotStatus->commandStatus = EMCMOT_COMMAND_BAD_EXEC;
					tpAbort(&emcmotInternal->coord_tp);
					SET_MOTION_ERROR_FLAG(1);
					break;
		        }
			    break;

    		case EMCMOT_SET_FREE:
    			rtapi_print_msg(RTAPI_MSG_DBG, "axis_num %d set free motion mode\n", axis_num);
    			emcmotInternal->freeing = 1;
    			emcmotInternal->coordinating = 0;
    			break;

    		case EMCMOT_SET_COORD:
    			rtapi_print_msg(RTAPI_MSG_DBG, "set coord motion mode\n");
    			emcmotInternal->coordinating = 1;
    			emcmotInternal->freeing = 0;
    			break;

    		//空闲
    		case EMCMOT_FREE:
    			break;

    		default:
    			break;
	    }
    }



	if (emcmotStatus->commandStatus != EMCMOT_COMMAND_OK) {
		rtapi_print_msg(RTAPI_MSG_DBG, "ERROR: %d\n",
		emcmotStatus->commandStatus);
	}

	/* synch tail count */
	emcmotStatus->tail = emcmotStatus->head;
	emcmotConfig->tail = emcmotConfig->head;
	emcmotInternal->tail = emcmotInternal->head;

}


void emcmotCommandHandler(void *arg, long servo_period)
{
    if (rtapi_mutex_try(&emcmotStruct->command_mutex) != 0) {
        // Failed to take the mutex, because it is held by Task.
        // This means Task is in the process of updating the command.
        // Give up for now, and try again on the next invocation.
        return;
    }

    emcmotCommandHandler_locked(arg, servo_period);
    rtapi_mutex_give(&emcmotStruct->command_mutex);

	// if(first_flag == 0)
	// {
	// 	clock_gettime(CLOCK_MONOTONIC, &now);
	// 	next = now;
	// 	first_flag = 1;
	// }
	//
	// clock_gettime(CLOCK_MONOTONIC, &now);
	// int64 nowstr = now.tv_sec*1000000000LL + now.tv_nsec;
	// int64 nextstr = next.tv_sec*1000000000LL + next.tv_nsec;
	//
	// int64 delay = abs(nowstr - nextstr - 1000000LL) / 1000;
	// total_val += delay;
	//
	// if (time_val % 10000 == 0) {
	// 	rtapi_print_msg(RTAPI_MSG_DBG, "vag = %d\n", total_val/per_val);
	// }
	// per_val++;
	// time_val++;
	// next = now;
}


void emcmotSetRotaryUnlock(int axisnum, int unlock)
{
	if (NULL == emcmot_hal_data->axis[axisnum].unlock)
	{
		rtapi_print_msg(RTAPI_MSG_ERR,
		"emcmotSetRotaryUnlock(): No unlock pin configured for axis %d\n"
		"   Use motmod parameter: unlock_joints_mask=%X",
		axisnum,1<<axisnum);
		return;
	}
	*(emcmot_hal_data->axis[axisnum].unlock) = unlock;
}

int emcmotGetRotaryIsUnlocked(int axisnum) {
	static int gave_message = 0;
	if (NULL == emcmot_hal_data->axis[axisnum].unlock) {
		if (!gave_message) {
			rtapi_print_msg(RTAPI_MSG_ERR,
			"emcmotGetRotaryUnlocked(): No unlock pin configured for axis %d\n"
			"   Use motmod parameter: unlock_joints_mask=%X'",
			axisnum,1<<axisnum);
		}
		gave_message = 1;
		return 0;
	}
	return *(emcmot_hal_data->axis[axisnum].is_unlocked);
}