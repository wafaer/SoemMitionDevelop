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

/* emcmotStruct : 主结构体指针，内含命令互斥锁（command_mutex） */
extern emcmot_struct_t *emcmotStruct;

/* emcmotCommand : 命令结构体指针，由 Task 层写入，Motion 层读取。*/
extern emcmot_command_t *emcmotCommand;

/* emcmotStatus : 状态结构体指针，由 Motion 层写入，Task 层读取。*/
extern emcmot_status_t *emcmotStatus;

/* emcmotInternal : 运动层内部数据结构指针，Motion 层内部使用。*/
extern emcmot_internal_t *emcmotInternal;

/* axes[] : 轴的运行时数据数组，共 EMCMOT_MAX_AXIS 个元素。*/
extern emcmot_axis_t axes[EMCMOT_MAX_AXIS];

/* emcmot_hal_data : HAL 引脚数据容器的指针。 */
extern emcmot_hal_data_t *emcmot_hal_data;

/*
 * joint_jog_ok — 检查轴是否可以执行点动操作
*/
static int joint_jog_ok(int axis_num, double vel)
{
    emcmot_axis_t *axis;
    int neg_limit_override, pos_limit_override;

    /* 步骤 1：根据轴编号获取轴数据结构指针 */
    axis = &axes[axis_num];

    /* 步骤 2：检查限位覆盖位掩码
     * overrideLimitMask 中每个轴占两位：
     *   - bit (axis_num*2)     : 是否忽略负向硬限位（值为 1 时忽略）
     *   - bit (axis_num*2 + 1) : 是否忽略正向硬限位（值为 2 时忽略）
     * 通过按位与操作提取对应轴的覆盖状态 */
    neg_limit_override = emcmotStatus->overrideLimitMask & ( 1 << (axis_num*2));
    pos_limit_override = emcmotStatus->overrideLimitMask & ( 2 << (axis_num*2));
    /* 如果两个方向都被覆盖（限位退出模式），直接允许点动 */
    if ( neg_limit_override && pos_limit_override )
    {
		return 1;
    }
    /* 步骤 3：轴编号有效性检查，防止数组越界 */
    if (axis_num < 0 || axis_num >= ALL_AXES)
    {
		return 0;
    }
    /* 步骤 4：硬限位检查
     * PHL_FLAG (Positive Hardware Limit): 正向硬限位标志
     * 当正向点动时，如果正向硬限位已被触发，则拒绝执行 */
    if (vel > 0.0 && GET_AXIS_PHL_FLAG(axis))
    {
		return 0;
    }
    /* 步骤 4（续）：负向硬限位检查
     * NHL_FLAG (Negative Hardware Limit): 负向硬限位标志
     * 当负向点动时，如果负向硬限位已被触发，则拒绝执行 */
    if (vel < 0.0 && GET_AXIS_NHL_FLAG(axis)) {
		return 0;
    }
    // refresh_jog_limits(axis,axis_num);
    /* 步骤 6：点动软限位检查
     * 如果当前轴位置已经超出点动允许范围，则拒绝点动
     * max_jog_limit 和 min_jog_limit 定义了点动操作的安全边界 */
    if ( vel > 0.0 && (axis->pos_cmd > axis->max_jog_limit) ) {
		return 0;
    }
    if ( vel < 0.0 && (axis->pos_cmd < axis->min_jog_limit) ) {
		return 0;
    }

    return 1;  /* 所有检查通过，允许点动 */
}


/*
 * refresh_jog_limits — 动态刷新轴的点动限位边界
 */
void refresh_jog_limits(emcmot_axis_t *axis, int axis_num)
{
    double range;  /* 机床全行程范围 */

    /* 计算全行程范围：正向软限位 - 负向软限位 */
	range = axis->max_pos_limit - axis->min_pos_limit;
	/* 动态刷新点动上限：当前位置 + 全行程范围 */
	axis->max_jog_limit = axis->pos_fb + range;
	/* 动态刷新点动下限：当前位置 - 全行程范围 */
	axis->min_jog_limit = axis->pos_fb - range;
}


/*
 * limits_ok — 检查所有轴的硬限位是否都处于安全状态
 */
static int limits_ok(void)
{
	int joint_num;
	emcmot_axis_t *axis;

	/* 遍历所有轴，检查每个活跃轴的硬限位状态 */
	for (joint_num = 0; joint_num < ALL_AXES; joint_num++)
	{
		axis = &axes[joint_num];
		/* 跳过非活跃轴（非使能轴不参与限位检查） */
		if (!GET_AXIS_ACTIVE_FLAG(axis)) {
			continue;
		}

		/* 如果任意一个活跃轴触发了正向或负向硬限位，立即返回不安全状态 */
		if (GET_AXIS_PHL_FLAG(axis) || GET_AXIS_NHL_FLAG(axis)) {
			return 0;
		}
	}

	return 1;  /* 所有活跃轴均未触发硬限位，返回安全状态 */
}

/*
 * emcmotCommandHandler_locked — 命令处理核心函数
 */
void emcmotCommandHandler_locked(void *arg, long servo_period)
{
    (void)arg;  /* 消除未使用参数警告 */
    int axis_num;  /* 当前处理的轴编号 */
    emcmot_axis_t *axis;  /* 当前处理的轴数据结构指针 */

    int32_t tmp1;  /* 临时变量：用于计算增量点动的目标位置 */
    char issue_atspeed = 0;  /* 主轴恒速标志：0=主轴未达到目标速度 */

    /* ===== 阶段一：新命令检测 =====
     * 通过比较命令序列号来检测是否有新命令到达。
     * commandNum 由 Task 层递增，commandNumEcho 由 Motion 层更新为已处理的最大序列号。
     * 只有两者不相等时，才说明有尚未处理的新命令。 */
    if (emcmotCommand->commandNum != emcmotStatus->commandNumEcho)
    {
        /* 更新环形缓冲区的写入指针（Head） */
        emcmotStatus->head++;
        emcmotInternal->head++;

        /* 命令回显：将命令内容和序列号写入状态缓冲区 */
        emcmotStatus->commandEcho = emcmotCommand->command;
        emcmotStatus->commandNumEcho = emcmotCommand->commandNum;

        /* 预设命令状态为成功，后续若有错误会覆盖此值 */
        emcmotStatus->commandStatus = EMCMOT_COMMAND_OK;

        /* 从命令结构体中提取目标轴编号 */
        axis_num = emcmotCommand->axis;

    	axis = &axes[axis_num];  /* 获取目标轴的数据结构指针 */

        /* ===== 阶段二：命令分发 =====
         * 根据命令类型（command 字段的枚举值）执行相应的处理逻辑 */
    	switch (emcmotCommand->command)
	    {
            /* ---------------------------------------------------- */
            /* case EMCMOT_DOWNLOADS_CONFIG — 下载配置参数         */
            /* ---------------------------------------------------- */
    		case EMCMOT_DOWNLOADS_CONFIG:
                /* 将命令中的加速度命令写入轴数据 */
    			axis->acc_cmd = emcmotCommand->acc;
                /* 将命令中的速度命令写入轴数据 */
    			axis->vel_cmd = emcmotCommand->vel;
                /* 设置轴的最大加速度限制 */
    			axis->acc_limit = emcmotCommand->Maxacc;
                /* 设置轴的最大速度限制 */
    			axis->vel_limit = emcmotCommand->Maxvel;
                /* 设置轴的正向软限位（机床行程边界） */
    			axis->max_pos_limit = emcmotCommand->maxLimit;
                /* 设置轴的负向软限位 */
    			axis->min_pos_limit = emcmotCommand->minLimit;
                /* 初始化点动限位为与软限位相同的值
                 * 注：refresh_jog_limits() 被注释掉，意味着点动限位使用静态值 */
    			axis->max_jog_limit = emcmotCommand->maxLimit;
    			axis->min_jog_limit = emcmotCommand->minLimit;
                /* 同步最大加速度到全局状态，供其他组件查询 */
    			emcmotStatus->Maxacc = emcmotCommand->Maxacc;
                /* 同步最大速度到全局状态 */
    			emcmotStatus->Maxvel = emcmotCommand->Maxvel;
                /* 设置坐标轨迹规划器的路径容差为 200 用户单位
                 * 容差用于 CONTINUOUS（连续路径）模式，
                 * 当轨迹偏差超过此值时会触发错误或重新规划 */
    			emcmotInternal->coord_tp.tolerance = 200;

                /* 同步到全局配置结构 */
    			emcmotConfig->maxAxisScale = emcmotCommand->maxAxisScale;
                /* 设置轨迹速度限值为最大速度 */
    			emcmotConfig->limitVel = emcmotCommand->Maxvel;
    			// refresh_jog_limits(axis,axis_num);

                /* 打印配置下载完成信息（INFO 级别） */
    			rtapi_print_msg(RTAPI_MSG_INFO, "axis %d DOWNLOADS_CONFIG\n",axis_num);
    			break;

            /* ---------------------------------------------------- */
            /* case EMCMOT_AXIS_ENABLE — 使能单个轴               */
            /* ---------------------------------------------------- */
    		case EMCMOT_AXIS_ENABLE:

    			rtapi_print_msg(RTAPI_MSG_DBG, "axis %d ENABLE\n",axis_num);
                /* 检查全局运动使能 HAL 引脚是否已置 1
                 * 这是安全联锁：仅当上位机明确使能后，才允许使能轴 */
    			if ( *(emcmot_hal_data->enable) == 0 )
    			{
                    /* 全局未使能时，打印警告但不做任何操作 */
    				rtapi_print_msg(RTAPI_MSG_DBG,"axis %d can't enable motion, enable input is false\n",axis_num);
    			} else
    			{
                    /* 全局已使能：写入轴的放大器使能 HAL 引脚，激活伺服 */
    				*(emcmot_hal_data->axis[axis_num].amp_enable) = 1;
                    /* 设置轴的活跃标志，表示该轴已激活 */
    				SET_AXIS_ACTIVE_FLAG(axis, 1);
    			}
    			break;

            /* ---------------------------------------------------- */
            /* case EMCMOT_AXIS_DISABLE — 去使能单个轴             */
            /* ---------------------------------------------------- */
    		case EMCMOT_AXIS_DISABLE:
    			rtapi_print_msg(RTAPI_MSG_DBG, "axis %d DISABLE\n",axis_num);
                /* 写入放大器使能引脚为 0，切断伺服输出 */
    			*(emcmot_hal_data->axis[axis_num].amp_enable) = 0;
                /* 清除轴的活跃标志 */
    			SET_AXIS_ACTIVE_FLAG(axis, 0);
    			break;

            /* ---------------------------------------------------- */
            /* case EMCMOT_MOTION_ENABLE — 使能运动子系统          */
            /* ---------------------------------------------------- */
    		case EMCMOT_MOTION_ENABLE:
    			rtapi_print_msg(RTAPI_MSG_DBG, "axis %d MOTION ENABLE\n",axis_num);
                /* 设置内部使能标志，这是全局运动使能的核心标志
                 * 伺服线程会检查此标志来决定是否执行运动指令 */
    			emcmotInternal->enabling = 1;
    			break;

            /* ---------------------------------------------------- */
            /* case EMCMOT_MOTION_DISABLE — 去使能运动子系统       */
            /* ---------------------------------------------------- */
    		case EMCMOT_MOTION_DISABLE:
    			rtapi_print_msg(RTAPI_MSG_DBG, "axis %d MOTION DISABLE\n",axis_num);
                /* 清除内部使能标志，立即停止所有运动 */
    			emcmotInternal->enabling = 0;
    			break;

            /* ---------------------------------------------------- */
            /* case EMCMOT_ABORT — 紧急停止所有运动                 */
            /* ---------------------------------------------------- */
	        case EMCMOT_ABORT:
	            rtapi_print_msg(RTAPI_MSG_DBG, "ABORT %d", axis_num);
    			/*停止规划器
    			 * 根据当前运动模式决定停止哪个规划器 */
	            if (GET_MOTION_COORD_FLAG())
	            {
                    /* Coord 模式：停止坐标轨迹规划器
                     * tpAbort 会停止 coord_tp 中所有排队的轨迹段 */
	                tpAbort(&emcmotInternal->coord_tp);
	            } else
	            {
                    /* Free 模式：遍历所有轴，分别停止每个轴的 free_tp */
	                for (axis_num = 0; axis_num < ALL_AXES; axis_num++)
	                {
	                    axis = &axes[axis_num];
	                    axis->free_tp.enable = 0;  /* 禁用自由轨迹规划器 */
	                }
	            }
    			/*更新轴状态
    			 * 清除所有错误和故障标志，重置运动子系统状态 */
	            SET_MOTION_ERROR_FLAG(0);  /* 清除全局运动错误标志 */
                /* 遍历所有轴，清除轴级错误和故障标志 */
	            for (axis_num = 0; axis_num < ALL_AXES; axis_num++)
	            {
	                axis = &axes[axis_num];
	                SET_AXIS_ERROR_FLAG(axis, 0);   /* 清除轴错误标志 */
	                SET_AXIS_FAULT_FLAG(axis, 0);   /* 清除轴故障标志 */
	            }
                /* 清除暂停标志 */
	            emcmotStatus->paused = 0;
	            break;

            /* ---------------------------------------------------- */
            /* case EMCMOT_PAUSE — 暂停坐标轨迹运动                 */
            /* ---------------------------------------------------- */
    		case EMCMOT_PAUSE:
    			rtapi_print_msg(RTAPI_MSG_DBG, "axis %d PAUSE\n",axis_num);
                /* 通知坐标轨迹规划器暂停
                 * 规划器会保存当前位置，下次 RESUME 时从该位置继续 */
    			tpPause(&emcmotInternal->coord_tp);
                /* 设置暂停标志，供 GUI 等组件查询显示 */
    			emcmotStatus->paused = 1;
    			break;

            /* ---------------------------------------------------- */
            /* case EMCMOT_JOG_CONT — 连续点动                      */
            /* ---------------------------------------------------- */
    		case EMCMOT_JOG_CONT:
    			rtapi_print_msg(RTAPI_MSG_DBG, "JOG_CONT\n");
    			rtapi_print_msg(RTAPI_MSG_DBG, " %d", axis_num);
                /* 前置条件检查：运动必须已使能 */
    			if (!GET_MOTION_ENABLE_FLAG()) {
    				rtapi_print_msg(RTAPI_MSG_DBG,("Can't jog joint when not enabled\n"));
    				SET_AXIS_ERROR_FLAG(axis, 1);  /* 设置轴错误标志 */
    				break;
    			}
    			/*检查禁止标志
    			 * jog_inhibit 是 HAL 引脚，可由外部设备（如安全PLC）置 1
    			 * 来强制禁止所有点动操作 */
    			if (*(emcmot_hal_data->jog_inhibit)){
    				rtapi_print_msg(RTAPI_MSG_DBG,"axis %d Cannot jog while jog-inhibit is active\n",axis_num);
    				break;
    			}

    			/*检查限位
    			 * joint_jog_ok 综合检查硬限位、软限位、覆盖标志等条件 */
    			if (!joint_jog_ok(axis_num, emcmotCommand->Maxvel)) {
    				SET_AXIS_ERROR_FLAG(axis, 1);
    				break;
    			}
    			/*设置点动目标位置
    			 * 连续点动的目标位置设为对应方向的限位边界，
    			 * 让轴持续运动直到碰到限位或松开按键 */
    			// refresh_jog_limits(axis,axis_num);
    			if (emcmotCommand->Maxvel > 0.0) {
                    /* 正向点动：目标位置设为正向点动限位 */
    				axis->free_tp.pos_cmd = axis->max_jog_limit;
    			} else {
                    /* 负向点动：目标位置设为负向点动限位 */
    				axis->free_tp.pos_cmd = axis->min_jog_limit;
    			}

    			/* set velocity of jog
    			 * 速度参数取轴配置值和命令值中较小的（保守优先） */
    			if (axis->vel_cmd < emcmotCommand->Maxvel)
    			{
    				axis->free_tp.vel = axis->vel_cmd;
    			}else
    			{
    				axis->free_tp.vel = emcmotCommand->Maxvel;
    			}
                /* 设置自由轨迹规划器的最大速度（用于规划计算） */
    			axis->free_tp.max_vel = emcmotCommand->Maxvel;
    			/* use max joint accel
    			 * 加速度参数同样取较小值 */
    			if (axis->acc_cmd < emcmotCommand->Maxacc)
    			{
    				axis->free_tp.acc = axis->acc_cmd;
    			}else
    			{
    				axis->free_tp.acc = emcmotCommand->Maxacc;
    			}
                /* 设置自由轨迹规划器的最大加速度（用于规划计算） */
    			axis->free_tp.max_acc = axis->acc_limit;
    			/* and let it go
    			 * 使能自由轨迹规划器，开始执行点动运动 */
    			axis->free_tp.enable = 1;

                /* 中止所有遥操作点动（来自 axis.c），
    			 * 防止连续点动和遥操作点动之间的冲突 */
    			axis_jog_abort_all(0);
                /* 清除轴错误标志 */
    			SET_AXIS_ERROR_FLAG(axis, 0);
    			break;

            /* ---------------------------------------------------- */
            /* case EMCMOT_JOG_INCR — 增量点动                      */
            /* ---------------------------------------------------- */
	        case EMCMOT_JOG_INCR:
			    //相对增量点动
			    rtapi_print_msg(RTAPI_MSG_DBG, "JOG_INCR %d\n", axis_num);
                /* 前置条件检查：运动必须已使能 */
			    if (!GET_MOTION_ENABLE_FLAG()) {
					rtapi_print_msg(RTAPI_MSG_DBG, "axis %d Can't jog joint when not enabled\n",axis_num);
					SET_AXIS_ERROR_FLAG(axis, 1);
					break;
			    }
    			/*如果禁止点动
    			 * 检查外部点动禁止信号 */
		        if (*(emcmot_hal_data->jog_inhibit)){
		               rtapi_print_msg(RTAPI_MSG_DBG, "axis %d Cannot jog while jog-inhibit is active\n",axis_num);
		           break;
		        }
    			/*检查限位
    			 * joint_jog_ok 检查轴是否可以执行点动 */
				if (!joint_jog_ok(axis_num, emcmotCommand->Maxvel))
				{
					SET_AXIS_ERROR_FLAG(axis, 1);
					break;
				}
    		    //计算目标位置，正向/负向偏移
    			/* set target position for jog
    			 * 根据 dir 标志（方向位）决定是加还是减 offset */
    			if (emcmotCommand->dir)
    			{
                    /* dir == 1: 正向运动，目标位置 = 当前位置 + 增量 */
    				tmp1 = axis->free_tp.pos_cmd + emcmotCommand->offset;
    			} else {
                    /* dir == 0: 负向运动，目标位置 = 当前位置 - 增量 */
    				tmp1 = axis->free_tp.pos_cmd - emcmotCommand->offset;
    			}
    		    // 刷新关节点动限位
    			/* don't jog past limits */
    			// refresh_jog_limits(axis,axis_num);
    			// 限位检查：如果计算出的目标位置超出限位，直接拒绝执行
    			if (tmp1 > axis->max_jog_limit) {
    				break;
    			}
    			if (tmp1 < axis->min_jog_limit) {
    				break;
    			}

    			// 执行点动
    			/* set target position
    			 * 将计算出的目标位置写入自由轨迹规划器 */
    			axis->free_tp.pos_cmd = tmp1;
    		    rtapi_print_msg(RTAPI_MSG_DBG, "cmd axis->free_tp.pos_cmd %d\n", axis->free_tp.pos_cmd);
    			/* set velocity of jog
    			 * 速度取较小值（保守优先） */
    			if (axis->vel_cmd < emcmotCommand->Maxvel)
    			{
    				axis->free_tp.vel = axis->vel_cmd;
    			}else
    			{
    				axis->free_tp.vel = emcmotCommand->Maxvel;
    			}

    			axis->free_tp.max_vel = emcmotCommand->Maxvel;
    			/* use max joint accel
    			 * 加速度取较小值 */
    			if (axis->acc_cmd < emcmotCommand->Maxacc)
    			{
    				axis->free_tp.acc = axis->acc_cmd;
    			}else
    			{
    				axis->free_tp.acc = emcmotCommand->Maxacc;
    			}
                /* 【BUG】重复赋值：无论分支如何，结果都被下一行覆盖。
    				 这一行应该删除。 */
    			axis->free_tp.acc = axis->acc_cmd;
    			axis->free_tp.max_acc = axis->acc_limit;
    			//set dir
                /* 设置自由轨迹规划器的方向标志 */
    			axis->free_tp.dir = emcmotCommand->dir;
    			/* and let it go
    			 * 使能自由轨迹规划器并中止遥操作点动 */
    			axis->free_tp.enable = 1;
    			axis_jog_abort(axis_num, 0);

				rtapi_print_msg(RTAPI_MSG_DBG, "axis->vel_cmd %d\n", axis->vel_cmd);

    			SET_AXIS_ERROR_FLAG(axis, 0);
			    break;

            /* ---------------------------------------------------- */
            /* case EMCMOT_JOG_ABS — 绝对点动                      */
            /* ---------------------------------------------------- */
			case EMCMOT_JOG_ABS:
			    /* do an absolute jog */
			    //绝对运动
			    rtapi_print_msg(RTAPI_MSG_DBG, "JOG_ABS"" %d\n", axis_num);
                /* 前置条件检查：运动必须已使能 */
			    if (!GET_MOTION_ENABLE_FLAG()) {
				rtapi_print_msg(RTAPI_MSG_DBG, "Can't jog joint when not enabled\n");
				SET_AXIS_ERROR_FLAG(axis, 1);
				break;
			    }
		        /*检查禁止标志
		         * 检查外部点动禁止信号 */
		        if (*(emcmot_hal_data->jog_inhibit)){
		               rtapi_print_msg(RTAPI_MSG_DBG,("Cannot jog while jog-inhibit is active\n"));
		            break;
		        }

    			/*检查限位
    			 * joint_jog_ok 综合检查 */
    			if (!joint_jog_ok(axis_num, emcmotCommand->Maxvel)) {
    				SET_AXIS_ERROR_FLAG(axis, 1);
    				break;
    			}

                /* 直接使用命令中的 offset 作为绝对目标位置
    			 * （与 JOG_INCR 的相对计算方式不同） */
    			axis->free_tp.pos_cmd = emcmotCommand->offset;
    			// refresh_jog_limits(axis,axis_num);
                /* 限位边界截断：如果目标超出限位，截断到最近的限位值
    			 * 与 JOG_INCR 的"直接拒绝"策略不同，这里是"截断执行" */
    			if (axis->free_tp.pos_cmd > axis->max_jog_limit) {
    				axis->free_tp.pos_cmd = axis->max_jog_limit;
    			}
    			if (axis->free_tp.pos_cmd < axis->min_jog_limit) {
    				axis->free_tp.pos_cmd = axis->min_jog_limit;
    			}
    			/* set velocity of jog
    			 * 速度取较小值 */
    			if (axis->vel_cmd < emcmotCommand->Maxvel)
    			{
    				axis->free_tp.vel = axis->vel_cmd;
    			}else
    			{
    				axis->free_tp.vel = emcmotCommand->Maxvel;
    			}
                /* 设置自由轨迹规划器的最大速度 */
    			axis->free_tp.max_vel = fabs(emcmotCommand->Maxvel);
    			/* use max joint accel
    			 * 加速度取较小值 */
    			if (axis->acc_cmd < emcmotCommand->Maxacc)
    			{
    				axis->free_tp.acc = axis->acc_cmd;
    			}else
    			{
    				axis->free_tp.acc = emcmotCommand->Maxacc;
    			}
                /* 设置自由轨迹规划器的最大加速度 */
    			axis->free_tp.max_acc = axis->acc_limit;
    			/* and let it go
    			 * 使能自由轨迹规划器 */
    			axis->free_tp.enable = 1;
    			SET_AXIS_ERROR_FLAG(axis, 0);
		        break;

            /* ---------------------------------------------------- */
            /* case EMCMOT_JOG_ABORT — 单轴点动中止                 */
            /* ---------------------------------------------------- */
    		case EMCMOT_JOG_ABORT:
                /* 【代码问题】axis 是指针，这里检查 axis == 0（NULL）没有实际意义
    			 * 正确的检查应该是 axis_num == 0。保留此检查可能是一个遗留 bug。 */
    			if (axis == 0) { break; }
                /* 禁用自由轨迹规划器，停止该轴的点动运动 */
    			axis->free_tp.enable = 0;
    			/* update status flags
    			 * 清除轴错误标志 */
    			SET_AXIS_ERROR_FLAG(axis, 0);
    			break;

            /* ---------------------------------------------------- */
            /* case EMCMOT_SET_LINE — 直线插补                     */
            /* ---------------------------------------------------- */
    		case EMCMOT_SET_LINE:
			    rtapi_print_msg(RTAPI_MSG_DBG, "SET_LINE\n");
                /* 前置条件：必须在 Coord 模式下且运动已使能 */
    			if (!GET_MOTION_COORD_FLAG() || !GET_MOTION_ENABLE_FLAG()) {
					rtapi_print_msg(RTAPI_MSG_DBG,("need to be enabled, in coord mode for linear move\n"));
					emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_COMMAND;  /* 命令无效 */
					SET_MOTION_ERROR_FLAG(1);  /* 设置运动错误标志 */
					break;
			    }/*检查是否超出限位
    			 * 直线运动只能在所有轴都未触发硬限位的情况下执行 */
    			else if (!limits_ok()) {
					rtapi_print_msg(RTAPI_MSG_DBG,("can't do linear move with limits exceeded\n"));
					emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_PARAMS;  /* 参数无效 */
					tpAbort(&emcmotInternal->coord_tp);  /* 停止坐标轨迹规划器 */
					SET_MOTION_ERROR_FLAG(1);
					break;
			    }

			    /* append it to the emcmotInternal->coord_tp
    			 * 为直线段分配 ID，用于后续状态查询和同步 */
			    tpSetId(&emcmotInternal->coord_tp, emcmotCommand->id);
                /* 处理参考坐标系（相对/绝对）
    			 * ref == 1: 相对坐标（G91 模式）
    			 * ref == 0: 绝对坐标（G90 模式） */
    			if (emcmotCommand->ref)
    			{
                    /* 相对坐标：目标 = 当前位置 + 命令中的增量 */
    				if (emcmotCommand->dir)
    				{
                        /* dir == 1: 正向增量 */
    					emcmotCommand->pos.tran.x = emcmotInternal->coord_tp.goalPos.tran.x + emcmotCommand->pos.tran.x;
    					emcmotCommand->pos.tran.y = emcmotInternal->coord_tp.goalPos.tran.y + emcmotCommand->pos.tran.y;
    					emcmotCommand->pos.tran.z = emcmotInternal->coord_tp.goalPos.tran.z + emcmotCommand->pos.tran.z;
    				}else
    				{
                        /* dir == 0: 负向增量 */
    					emcmotCommand->pos.tran.x = emcmotInternal->coord_tp.goalPos.tran.x - emcmotCommand->pos.tran.x;
    					emcmotCommand->pos.tran.y = emcmotInternal->coord_tp.goalPos.tran.y - emcmotCommand->pos.tran.y;
    					emcmotCommand->pos.tran.z = emcmotInternal->coord_tp.goalPos.tran.z - emcmotCommand->pos.tran.z;
    				}
    			}

                /* 设置轨迹速度限值 */
    			tpSetVlimit(&emcmotInternal->coord_tp, emcmotConfig->limitVel);

			    /* 调用坐标轨迹规划器添加直线段
    			 * tpAddLine 会将直线段加入规划器的队列中 */
			    int res_addline = tpAddLine(&emcmotInternal->coord_tp,
							emcmotCommand->pos,
							emcmotCommand->motion_type,
							emcmotCommand->vel,
							emcmotCommand->Maxvel,
							emcmotCommand->acc,
							emcmotStatus->enables_new,
							issue_atspeed,
							emcmotCommand->turn);
				/*直线添加失败
    			 * tpAddLine 返回负值表示添加失败（队列满、参数错误等） */
    			if (res_addline >= 0) {
    				rtapi_print_msg(RTAPI_MSG_INFO,
						"[INTERP START] LINE queued: id=%d, target=(%.3f, %.3f, %.3f), vel=%.3f\n",
						emcmotCommand->id,
						emcmotCommand->pos.tran.x,
						emcmotCommand->pos.tran.y,
						emcmotCommand->pos.tran.z,
						emcmotCommand->vel);
    			}
		        if (res_addline < 0) {
		            rtapi_print_msg(RTAPI_MSG_DBG,("can't add linear move at line %d, error code %d"),
		                    emcmotCommand->id, res_addline);
		            emcmotStatus->commandStatus = EMCMOT_COMMAND_BAD_EXEC;  /* 执行失败 */
		            tpAbort(&emcmotInternal->coord_tp);  /* 停止规划器 */
		            SET_MOTION_ERROR_FLAG(1);
		            break;
		        }

                /* 设置主轴同步参数（如果有主轴联动） */
    			tpSetSpindleSync(&emcmotInternal->coord_tp,emcmotCommand->spindle, emcmotCommand->spindlesync, 0);
                /* 通知所有轴的三次样条插值器准备下一数据点
    			 * cubic.needNextPoint 标志触发 cubic.c 中的插值计算 */
    			for (int t = 0; t < EMCMOT_MAX_AXIS; t++) {
    				axes[t].cubic.needNextPoint = 1;
    			}

			    break;

            /* ---------------------------------------------------- */
            /* case EMCMOT_SET_CIRCLE — 圆弧插补                   */
            /* ---------------------------------------------------- */
			case EMCMOT_SET_CIRCLE:
			    rtapi_print_msg(RTAPI_MSG_DBG, "SET_CIRCLE\n");
                /* 前置条件检查：Coord 模式 + 运动使能 + 无越限 */
			    if (!GET_MOTION_COORD_FLAG() || !GET_MOTION_ENABLE_FLAG()) {
					rtapi_print_msg(RTAPI_MSG_DBG,("need to be enabled, in coord mode for circular move\n"));
					emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_COMMAND;
					SET_MOTION_ERROR_FLAG(1);
					break;
			    }/*检查限位
    			 * 圆弧运动同样需要所有轴都在安全范围内 */
    			else if (!limits_ok()) {
					rtapi_print_msg(RTAPI_MSG_DBG,("can't do circular move with limits exceeded\n"));
					emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_PARAMS;
					tpAbort(&emcmotInternal->coord_tp);
					SET_MOTION_ERROR_FLAG(1);
					break;
			    }

			    /* append it to the emcmotInternal->coord_tp
    			 * 为圆弧段分配 ID */
			    tpSetId(&emcmotInternal->coord_tp, emcmotCommand->id);
                /* 调用坐标轨迹规划器添加圆弧段
    			 * 圆弧参数包括：终点、圆心、法向量、圈数等 */
			    int res_addcircle = tpAddCircle(&emcmotInternal->coord_tp, emcmotCommand->pos,
		                            emcmotCommand->center, emcmotCommand->normal,
		                            emcmotCommand->turn, emcmotCommand->motion_type,
		                            emcmotCommand->Maxvel, emcmotCommand->ini_maxvel,
		                            emcmotCommand->Maxacc, emcmotStatus->enables_new,
									issue_atspeed);

    			if (res_addcircle >= 0) {
    				rtapi_print_msg(RTAPI_MSG_INFO,
						"[INTERP START] CIRCLE queued: id=%d, center=(%.3f, %.3f, %.3f), turn=%d\n",
						emcmotCommand->id,
						emcmotCommand->center.x,
						emcmotCommand->center.y,
						emcmotCommand->center.z,
						emcmotCommand->turn);
    			}

                /* 处理添加失败 */
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

            /* ---------------------------------------------------- */
            /* case EMCMOT_SET_FREE — 切换到 Free 模式            */
            /* ---------------------------------------------------- */
    		case EMCMOT_SET_FREE:
    			rtapi_print_msg(RTAPI_MSG_DBG, "axis_num %d set free motion mode\n", axis_num);
                /* 设置自由模式标志，清除坐标模式标志 */
    			emcmotInternal->freeing = 1;
    			emcmotInternal->coordinating = 0;
    			break;

            /* ---------------------------------------------------- */
            /* case EMCMOT_SET_COORD — 切换到 Coord 模式          */
            /* ---------------------------------------------------- */
    		case EMCMOT_SET_COORD:
    			rtapi_print_msg(RTAPI_MSG_DBG, "set coord motion mode\n");
                /* 设置坐标模式标志，清除自由模式标志 */
    			emcmotInternal->coordinating = 1;
    			emcmotInternal->freeing = 0;
    			break;

            /* ---------------------------------------------------- */
            /* case EMCMOT_FREE — 空闲命令                        */
            /* ---------------------------------------------------- */
    		//空闲
    		case EMCMOT_FREE:
    			break;  /* 不执行任何操作 */

    		default:
    			break;  /* 未知命令，忽略 */
	    }
    }

    /* ===== 阶段三：状态同步（尾同步）=====
     * 无论是否有新命令，都执行以下状态报告工作 */

	/* 如果命令状态不是 OK，打印错误信息 */
	if (emcmotStatus->commandStatus != EMCMOT_COMMAND_OK) {
		rtapi_print_msg(RTAPI_MSG_DBG, "ERROR: %d\n",
		emcmotStatus->commandStatus);
	}

	/* synch tail count
	 * 将 Tail（读指针）同步到 Head（写指针）位置
	 * 这表示 NML 缓冲区中的数据已被消费，可以继续写入新数据 */
	emcmotStatus->tail = emcmotStatus->head;
	emcmotConfig->tail = emcmotConfig->head;
	emcmotInternal->tail = emcmotInternal->head;

}


/*
 * emcmotCommandHandler — 命令处理入口函数
 */
void emcmotCommandHandler(void *arg, long servo_period)
{
    /* 尝试获取命令互斥锁
     * rtapi_mutex_try 是非阻塞获取：如果锁被占用则立即返回失败
     * 这确保了 Motion 层不会被 Task 层阻塞 */
    if (rtapi_mutex_try(&emcmotStruct->command_mutex) != 0) {
        // Failed to take the mutex, because it is held by Task.
        // This means Task is in the process of updating the command.
        // Give up for now, and try again on the next invocation.
        return;  /* 锁被占用，跳过本次处理，等待下一周期重试 */
    }

    /* 成功获取锁：执行实际的命令处理（在锁保护下） */
    emcmotCommandHandler_locked(arg, servo_period);
    /* 处理完成：释放互斥锁，允许 Task 层继续写入命令 */
    rtapi_mutex_give(&emcmotStruct->command_mutex);
}

/*
 * emcmotSetRotaryUnlock — 设置旋转轴的解锁状态
 */
void emcmotSetRotaryUnlock(int axisnum, int unlock)
{
    /* 检查 unlock 引脚是否已配置（需要在 motmod insmod 时指定 unlock_joints_mask） */
	if (NULL == emcmot_hal_data->axis[axisnum].unlock)
	{
		rtapi_print_msg(RTAPI_MSG_ERR,
		"emcmotSetRotaryUnlock(): No unlock pin configured for axis %d\n"
		"   Use motmod parameter: unlock_joints_mask=%X",
		axisnum,1<<axisnum);
		return;
	}
    /* 写入解锁控制值到 HAL 引脚 */
	*(emcmot_hal_data->axis[axisnum].unlock) = unlock;
}


/*
 * emcmotGetRotaryIsUnlocked — 查询旋转轴是否已解锁
 */
int emcmotGetRotaryIsUnlocked(int axisnum) {
    /* static 变量：记录是否已经打印过错误消息（避免重复打印） */
	static int gave_message = 0;
    /* 检查 unlock 引脚是否已配置 */
	if (NULL == emcmot_hal_data->axis[axisnum].unlock) {
        /* 只在第一次检测到未配置时打印错误消息 */
		if (!gave_message) {
			rtapi_print_msg(RTAPI_MSG_ERR,
			"emcmotGetRotaryUnlocked(): No unlock pin configured for axis %d\n"
			"   Use motmod parameter: unlock_joints_mask=%X'",
			axisnum,1<<axisnum);
		}
		gave_message = 1;  /* 标记已打印，后续不再重复 */
		return 0;  /* 返回锁定状态 */
	}
    /* 读取实际的轴锁定状态 HAL 引脚 */
	return *(emcmot_hal_data->axis[axisnum].is_unlocked);
}
