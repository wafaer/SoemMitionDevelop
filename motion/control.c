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

/* emcmotStatus : 状态结构体指针，Motion 层向 Task 层报告状态的通道。
 * 本模块负责向此结构体写入各轴的位置、速度、误差、状态标志等信息。 */
extern emcmot_status_t *emcmotStatus;

/* emcmot_hal_data : HAL 引脚数据容器的指针。
 * 本模块通过此指针读取 HAL 输入（限位开关、故障信号等），
 * 并向其中写入 HAL 输出（位置命令、速度命令等）。 */
extern emcmot_hal_data_t *emcmot_hal_data;

/* emcmotInternal : 运动层内部数据结构。*/
extern emcmot_internal_t *emcmotInternal;

/* emcmotConfig : 系统配置参数（INI 文件中的 [AXIS_X] 等配置）。 */
extern emcmot_config_t *emcmotConfig;

/* axes[] : 轴的运行时数据数组，共 EMCMOT_MAX_AXIS 个元素（当前配置为 3 个：X/Y/Z）。
 * 每个元素包含该轴的位置、速度、加速度、限位、规划器等全部运行时数据。 */
extern emcmot_axis_t axes[EMCMOT_MAX_AXIS];

/* emcecatStatus : EtherCAT 总线状态（外部变量，由 EtherCAT 从站驱动写入）。 */
extern  ecat_status_t *emcecatStatus;

/*
 * pcmd_p[] — 笛卡尔位置命令的指针数组
 */
static double *pcmd_p[EMCMOT_MAX_AXIS];

/*
 * servo_period — 伺服周期
 */
static int32_t servo_period;

/*
 * last_period — 上一次伺服周期长度
 */
static unsigned long last_period = 0;

/*
 * coord_cubic_active — 坐标模式三次样条插值激活标志
 */
static bool   coord_cubic_active = 0;

/*
 * ext_offset_coord_limit — 外部偏移坐标越限标志
 */
static int    ext_offset_coord_limit  = 0;

/*
 * timest — 伺服线程执行周期计数器
 */
static int timest = 0;

/*
 * atomic_targetpos[] — 各轴的目标位置（原子写入）
 */
extern atomic_int_fast32_t atomic_targetpos[EMCMOT_MAX_AXIS];

/*
 * atomic_actpos[] — 各轴的实际位置（原子写入）
 */
extern atomic_int_fast32_t atomic_actpos[EMCMOT_MAX_AXIS];

static void handle_interpolation(void);
static void process_inputs(void);
static void check_for_faults(void);
static void set_operating_mode(void);
static void get_pos_cmds(long period);
static void output_to_hal(void);
static void update_status(void);

/*
 * emcmotController — 运动控制器主入口
 */
void emcmotController(void *arg, long period)
{

    (void)arg;  /* 消除未使用参数警告 */
    static int do_once = 1;  /* 初始化标志：仅在第一次调用时执行一次 */
	emcmot_axis_t *axis;
    /* ===== 阶段 0：初始化（仅第一次调用时执行）=====
     * 将 pcmd_p 指针数组与 emcmotStatus->carte_pos_cmd 的各分量绑定。
     * 这使得后续函数可以通过 pcmd_p[] 直接修改笛卡尔位置命令。 */
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
        do_once = 0;  /* 清除初始化标志，后续调用不再进入此块 */
    }

    /* ===== 阶段 0：伺服周期计时 =====
     * 通过 CPU 时钟计数器计算本次调用的实际时间间隔 */
    static long long int last = 0;  /* 上次调用时的 CPU 时钟值 */
	axis = &axes[0];  /* 获取第一个轴的数据（用于后续的一些计算） */
    long long int now = rtapi_get_clocks();  /* 获取当前 CPU 时钟计数器值 */
    long int this_run = (long int)(now - last);  /* 计算本次调用的时间间隔（CPU 时钟周期） */
    *(emcmot_hal_data->last_period) = this_run;  /* 写入 HAL 引脚，供外部监控 */
#ifdef HAVE_CPU_KHZ
    /* 如果定义了 cpu_khz 宏，计算实际的时间间隔（纳秒）并写入 HAL 引脚 */
    *(emcmot_hal_data->last_period_ns) = this_run * 1e6 / cpu_khz;
#endif

    last = now;  /* 更新 last，为下一次调用做准备 */

    /* calculate servo period as a double - period is in integer nsec
     * 将纳秒转换为秒，作为轨迹规划器的时间步长。
     * 例如：period = 1000000 ns → servo_period = 0.001 s（1ms）*/
    servo_period = period * 0.000000001;
	/* 防御性检查：如果计算结果异常小（可能是 period==0），回退到 1 */
	if(servo_period < 1)
	{
		servo_period = 1;
	}

    /* 如果伺服周期发生了变化，需要重新配置周期相关参数 */
    if (period != (long)last_period) {
        emcmotSetCycleTime(period);  /* 重新设置伺服周期 */
        last_period = period;
    }

    /* 更新状态环形缓冲区的写入指针，表示有新工作正在进行 */
    emcmotStatus->head++;
    /* here begins the core of the controller */

    /* 阶段 1：处理轨迹插补同步
     * 当运动模式切换到 Coord 时，同步坐标轨迹规划器的初始位置 */
	handle_interpolation();

	//处理轴数据，位置反馈、误差计算和限位检测
    /* 阶段 2：从 HAL 读取输入，处理反馈数据，计算跟随误差，检测限位开关 */
    process_inputs();
	//检查故障
    /* 阶段 3：检测各种故障条件（轴故障、跟随误差超限、misc error 等） */
    check_for_faults();

	//设置运动模式
    /* 阶段 4：根据 enabling/coordinating/freeing 标志设置运动模式 */
	set_operating_mode();

    /* 阶段 5：执行运动计算，生成位置命令
     * 根据当前运动模式（Free/Coord/Disabled），调用对应的轨迹规划器，
     * 计算每个轴的目标位置、速度和加速度 */
    get_pos_cmds(period);

    /* 阶段 6：将计算结果输出到 HAL 引脚，供伺服驱动器和上位机读取 */
    output_to_hal();

    /* 阶段 7：更新状态结构体，供 Task 层查询 */
    update_status();
    /* here ends the core of the controller */
    /* 心跳计数器递增，供上位机检测伺服线程是否存活 */
    emcmotStatus->heartbeat++;
    /* set tail to head, to indicate work complete
     * 同步 tail 指针，表示本周期工作完成，Task 层可以读取最新状态 */
    emcmotStatus->tail = emcmotStatus->head;
/* end of controller function */
}

/*
 * handle_interpolation — 处理轨迹规划器的模式切换同步
 */
static void handle_interpolation(void)
{
	static int prev_state = -1;  /* 上一次的运动状态，初始为 -1（不可能值） */
	/* 检测运动状态是否发生了变化 */
	if (emcmotStatus->motion_state != prev_state)
	{
		/* 当运动状态切换到坐标联动模式时，执行初始化同步 */
		if (emcmotStatus->motion_state == EMCMOT_MOTION_COORD)
		{
			EmcPose init_pose;  /* 初始化位置结构体 */
			/* 读取当前各轴的位置，构建初始坐标 */
			init_pose.tran.x = axes[0].pos_cmd;
			init_pose.tran.y = axes[1].pos_cmd;
			init_pose.tran.z = axes[2].pos_cmd;

			/* 将坐标轨迹规划器的初始位置设置为当前实际位置
			 * 这是关键的同步步骤：确保规划器从正确的起点开始 */
			tpSetPos(&emcmotInternal->coord_tp, &emcmotStatus->carte_pos_cmd);
		}

		/* 更新 prev_state，记录当前状态，为下一次比较做准备 */
		prev_state = emcmotStatus->motion_state;
	}
}

/*
 * process_inputs — 读取并处理所有轴的 HAL 输入\
 */
static void process_inputs(void)
{
    int axis_num;
    int32_t abs_ferror, scale;  /* abs_ferror: 跟随误差绝对值; scale: 比例因子 */
    axis_hal_t *axis_data;  /* 指向轴的 HAL 数据结构的指针 */
    emcmot_axis_t *axis;     /* 指向轴的内部运行时数据的指针 */
    unsigned char enables;    /* 使能标志集合（FS_ENABLED 等）*/

	//根据运动状态选择使能信号的来源
    /* compute net feed and spindle scale factors */
    /* 在 Coord 模式下，使用轨迹规划器排队时的使能标志（运动中途不能改变）。
     * 在 Free 模式下，使用当前的使能标志（实时响应）。 */
    if ( emcmotStatus->motion_state == EMCMOT_MOTION_COORD ) {
	enables = emcmotStatus->enables_queued;
    } else {
	enables = emcmotStatus->enables_new;
    }

    scale = 1;  /* 初始化比例因子为 1（100%） */
    /* 计算进给速率比例因子（Feedrate Override）
     * 只有在非 Free 模式且 FS_ENABLED 标志为 1 时才应用比例缩放 */
    if (   (emcmotStatus->motion_state != EMCMOT_MOTION_FREE)
        && (enables & FS_ENABLED) ) {
        /* 根据运动类型选择不同的比例因子 */
        if (emcmotStatus->motionType == 1) {
            /* 快速移动（G0）：使用 rapid_scale（快速倍率） */
            scale *= emcmotStatus->rapid_scale;
        } else
        {
            /* 进给移动（G1/G2/G3）：使用 feed_scale（进给倍率） */
            scale *= emcmotStatus->feed_scale;
        }
    }

	//读取并处理每个轴的输入
    /* 遍历所有轴，依次处理每个轴的输入数据 */
    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS ; axis_num++)
    {
		/* point to axis HAL data */
		axis_data = &(emcmot_hal_data->axis[axis_num]);
		/* point to axis data */
		axis = &axes[axis_num];
		/* 非活跃轴跳过，不参与任何处理 */
		if (!GET_AXIS_ACTIVE_FLAG(axis)) {
		    /* if joint is not active, skip it */
		    continue;
		}

		/* 从 HAL 引脚读取伺服驱动器反馈的电机位置（含补偿） */
		axis->motor_pos_fb = *(axis_data->motor_pos_fb);
	    axis->pos_fb = axis->motor_pos_fb - axis->motor_offset;
	    axis->ferror = axis->pos_cmd - axis->pos_fb;
		abs_ferror = (int32_t)fabs(axis->ferror);  /* 取绝对值用于比较 */
		if (axis->vel_limit > 0) {
			/* 计算：max_ferror * (当前速度 / 最大速度) */
			int64_t temp = (int64_t)axis->max_ferror * axis->vel_cmd / axis->vel_limit;
			// rtapi_print_msg(RTAPI_MSG_DBG, "temp %ld", temp);

		    /* 将计算结果（int64_t）赋给 ferror_limit（根据类型可能是 double 或 int32） */
		    axis->ferror_limit = temp;
			// rtapi_print_msg(RTAPI_MSG_DBG, " axis->ferror_limit %d\n", axis->ferror_limit);

		} else {
		    /* 速度限制为 0 时，将跟随误差限值也设为 0 */
		    axis->ferror_limit = 0;
		}
		/* 确保误差限值不低于最小跟随误差阈值 */
		if (axis->ferror_limit < axis->min_ferror) {
		    axis->ferror_limit = axis->min_ferror;
		}

		if (abs_ferror > axis->ferror_limit)
		{
		    SET_AXIS_FERROR_FLAG(axis, 1);  /* 设置跟随误差超限标志 */
		} else {
		    SET_AXIS_FERROR_FLAG(axis, 0);  /* 清除跟随误差超限标志 */
		}

    	// 读取正负限位限位开关
		/* 从 HAL 引脚读取物理限位开关的状态。
		 * 限位开关通常安装在行程末端，由机械挡块触发。 */
		if (*(axis_data->pos_lim_sw)) {
		    SET_AXIS_PHL_FLAG(axis, 1);  /* 正向（Positive）硬限位触发 */
		} else {
		    SET_AXIS_PHL_FLAG(axis, 0);  /* 正向硬限位未触发 */
		}
		if (*(axis_data->neg_lim_sw)) {
		    SET_AXIS_NHL_FLAG(axis, 1);  /* 负向（Negative）硬限位触发 */
		} else {
		    SET_AXIS_NHL_FLAG(axis, 0);  /* 负向硬限位未触发 */
		}
		/* 同时更新轴结构体中的 on_pos_limit 和 on_neg_limit 字段，
		 * 供后续逻辑查询 */
		axis->on_pos_limit = GET_AXIS_PHL_FLAG(axis);
		axis->on_neg_limit = GET_AXIS_NHL_FLAG(axis);

    	// 读取伺服驱动故障输入
    	/* 从 HAL 引脚读取伺服驱动器的故障信号。
    	 * amp_fault 由伺服驱动器在检测到过流、过压、通信中断等故障时置位。 */
    	if (*(axis_data->amp_fault)) {
    		SET_AXIS_FAULT_FLAG(axis, 1);  // 故障
    	} else {
    		SET_AXIS_FAULT_FLAG(axis, 0);  // 正常
    	}

    }
	// 如果点动进行中且收到停止请求，则停止点动
    // if jog in progress stop the jog if requested 点动
    /* 当点动运动正在进行，且收到了停止请求（正常停止或急停）时，
     * 调用 axis_jog_abort_all() 中止所有点动。
     * jog_stop_immediate 参数决定是急停（立即归零速度）还是正常减速停止。 */
    if (enables & *(emcmot_hal_data->jog_is_active) && (*(emcmot_hal_data->jog_stop) || *(emcmot_hal_data->jog_stop_immediate)))
    {
        axis_jog_abort_all(*(emcmot_hal_data->jog_stop_immediate));
    }

	timest++;  /* 伺服周期计数器递增 */
}

/*
 * check_for_faults — 检测各种故障条件并执行安全联锁
 */
static void check_for_faults(void)
{
    int axis_num, error_num;
    emcmot_axis_t *axis;
    int neg_limit_override, pos_limit_override;  /* 限位覆盖标志 */

	//检查全局使能
    /* 只有在运动启用状态下才检查全局使能信号。
     * 这是合理的设计：运动已经禁用时，使能引脚的变化没有意义。 */
    if ( GET_MOTION_ENABLE_FLAG() != 0 ) {
	/* 如果运动当前是启用状态，检查 HAL 全局使能引脚。
	 * 如果引脚从 1 变为 0（外部急停或安全联锁触发），立即关闭运动使能 */
	if ( *(emcmot_hal_data->enable) == 0 ) {
	    emcmotInternal->enabling = 0;  /* 关闭运动使能 */
	}
    }

	//检查轴故障
    /* 遍历所有轴，检查各类轴级故障条件 */
    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++) {
	/* point to joint data */
	axis = &axes[axis_num];
	/* 只有活跃且已使能的轴才参与故障检查 */
	if ( GET_AXIS_ACTIVE_FLAG(axis) && GET_AXIS_ENABLE_FLAG(axis) ) {
	    /* 轴限位是否被覆盖（jogging override）*/
	    neg_limit_override = emcmotStatus->overrideLimitMask & ( 1 << (axis_num*2));
	    pos_limit_override = emcmotStatus->overrideLimitMask & ( 2 << (axis_num*2));

	    /* 如果任意一个方向的限位被覆盖，设置轴错误并关闭使能 */
	    if (pos_limit_override || neg_limit_override )
	    {
	    	if (!GET_AXIS_ERROR_FLAG(axis)) {
	    		rtapi_print_msg(RTAPI_MSG_DBG, "joint %d on limit switch error\n", axis_num);
	    	}
	    	SET_AXIS_ERROR_FLAG(axis, 1);
	    	emcmotInternal->enabling = 0;  /* 关闭运动使能 */
	    }
		//伺服驱动故障检查
	    /* 检查伺服驱动器故障信号（过流、过压、通信中断等）*/
	    if (GET_AXIS_FAULT_FLAG(axis)) {
			/* 打印错误消息（仅一次，避免日志刷屏）*/
			if (!GET_AXIS_ERROR_FLAG(axis)) {
			    rtapi_print_msg(RTAPI_MSG_DBG, "joint %d amplifier fault", axis_num);
			}
			SET_AXIS_ERROR_FLAG(axis, 1);
			emcmotInternal->enabling = 0;  /* 关闭运动使能 */
	    }

		//跟随误差检查

	    /* 检查跟随误差是否超过限值。
	     * 跟随误差超限通常表示伺服系统跟不上指令（负载过大或参数不当）*/
	    if (GET_AXIS_FERROR_FLAG(axis)) {
			if (!GET_AXIS_ERROR_FLAG(axis))
			{
			    rtapi_print_msg(RTAPI_MSG_DBG, "axis %d following error\n", axis_num);
				print_msg(RTAPI_MSG_DBG, "axis %d following error\n", axis_num);
			}
			SET_AXIS_ERROR_FLAG(axis, 1);
			emcmotInternal->enabling = 0;  /* 关闭运动使能 */
	    }
	}
    }

    /* 检查杂项错误（misc_error 数组）。
     * 这些是用户可配置的通用错误输入，可连接外部故障信号。 */
    for (error_num=0; error_num < emcmotConfig->numMiscError; error_num++){
      if(emcmotStatus->misc_error[error_num] && GET_MOTION_ENABLE_FLAG()) {
        rtapi_print_msg(RTAPI_MSG_DBG, "Motion Stopped by misc error %d\n", error_num);
        emcmotInternal->enabling = 0;  /* 关闭运动使能 */
      }
    }
}

/*
 * set_operating_mode — 设置运动模式（使能/禁用、Free/Coord）
 */
static void set_operating_mode(void)
{
    int axis_num;
    emcmot_axis_t *axis;

	//motion disable 使能
    /* 【阶段一】处理运动禁用（Disable）
     * 触发条件：内部要求禁用 && 当前处于启用状态
     * 这通常由 check_for_faults() 检测到故障后触发（enabling = 0）*/
    if (!emcmotInternal->enabling && GET_MOTION_ENABLE_FLAG())
    {
    	rtapi_print_msg(RTAPI_MSG_DBG, "axis disenable\n");
		//清空轨迹
		/* 清空坐标轨迹规划器中的所有排队轨迹段 */
		tpClear(&emcmotInternal->coord_tp);
		/* 遍历所有轴，执行禁用操作 */
		for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++)
		{
		    axis = &axes[axis_num];
		    /* 停止自由轨迹规划器 */
			//去使能
		    axis->free_tp.enable = 0;
		    axis->free_tp.curr_vel = 0.0;  /* 速度归零 */
			//清空插补数据
		    /* 排空三次样条插值器，停止插值计算 */
		    cubicDrain(&(axis->cubic));
		    /* 对于活跃轴，标记为到位并清除使能标志 */
		    if (GET_AXIS_ACTIVE_FLAG(axis))
		    {
				SET_AXIS_INPOS_FLAG(axis, 1);  /* 标记为到位 */
				SET_AXIS_ENABLE_FLAG(axis, 0);  /* 清除使能标志 */
		    }
		}

    	//中止所有点动运动
	    /* axis_jog_abort_all(1) 中的参数 1 表示"急停"模式（立即归零速度）*/
	    axis_jog_abort_all(1);
		SET_MOTION_ENABLE_FLAG(0);  /* 清除全局使能标志 */
    }

    //motion 使能
    /* 【阶段二】处理运动启用（Enable）
     * 触发条件：内部要求启用 && 当前处于禁用状态
     * 这通常由 Task 层发送 MOTION_ENABLE 命令后触发（enabling = 1）*/
    if (emcmotInternal->enabling && !GET_MOTION_ENABLE_FLAG())
    {
    	rtapi_print_msg(RTAPI_MSG_DBG, "axis enable\n");
        /* 如果外部偏移曾经越限，清除受限标志 */
        if (*(emcmot_hal_data->eoffset_limited))
        {
            *(emcmot_hal_data->eoffset_limited) = 0;
        }
        /* 初始化所有轴的外部偏移状态 */
        axis_initialize_external_offsets();
        /* 同步坐标轨迹规划器的初始位置为当前笛卡尔位置 */
        tpSetPos(&emcmotInternal->coord_tp, &emcmotStatus->carte_pos_cmd);
		/* 遍历所有轴，同步位置并使能 */
		for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++)
		{
		    /* point to joint data */
		    axis = &axes[axis_num];
		    /* 将自由规划器的当前位置设置为当前关节位置（同步）*/
		    axis->free_tp.curr_pos = axis->pos_cmd;
		    /* 使能活跃轴 */
		    if (GET_AXIS_ACTIVE_FLAG(axis))
		    {
				SET_AXIS_ENABLE_FLAG(axis, 1);
		    }
		    /* 清除该轴的所有错误标志，允许运动 */
		    SET_AXIS_ERROR_FLAG(axis, 0);
		}

		SET_MOTION_ENABLE_FLAG(1);  /* 设置全局使能标志 */
		SET_MOTION_ERROR_FLAG(0);  /* 清除全局运动错误标志 */
	}

	//插补模式
	/* 【阶段三】处理切换到 Coord 模式
	 * 触发条件：请求进入 Coord && 当前不在 Coord 模式
	 * 前置条件：所有轴必须处于到位状态（INPOS）*/
	if (emcmotInternal->coordinating && !GET_MOTION_COORD_FLAG())
	{
		/* 只有所有轴都到位（运动已完成）时才允许切换模式 */
		if (GET_MOTION_INPOS_FLAG()) {
			rtapi_print_msg(RTAPI_MSG_DBG, "set coord\n");
			/* 从笛卡尔位置中减去外部偏移，恢复原始坐标系。
			 * 这确保 coord_tp 在进入 Coord 模式时使用不含偏移的位置作为起点。 */
			axis_apply_ext_offsets_to_carte_pos(-1, pcmd_p);

			/* 将坐标轨迹规划器的当前位置设置为当前的笛卡尔位置 */
			tpSetPos(&emcmotInternal->coord_tp, &emcmotStatus->carte_pos_cmd);
			/* 排空所有轴的三次样条插值器。
			 * 这确保在切换到 Coord 模式后，插值器从正确的起点重新开始。*/
			for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++) {
				axis = &axes[axis_num];
				cubicDrain(&(axis->cubic));
			}
			/* 清除限位覆盖标志，进入 Coord 模式后不允许限位覆盖 */
			emcmotInternal->overriding = 0;
			emcmotStatus->overrideLimitMask = 0;
			SET_MOTION_COORD_FLAG(1);  /* 设置坐标联动模式标志 */
			SET_MOTION_ERROR_FLAG(0);
		} else {
			/* 轴尚未到位，拒绝模式切换，清除 Coord 请求标志 */
			emcmotInternal->coordinating = 0;
		}
	}

	//单轴运动
	/* 【阶段四】处理切换到 Free 模式
	 * 触发条件：请求进入 Free && 当前不在 Free 模式
	 * 前置条件：所有轴必须处于到位状态（INPOS）*/
	if (emcmotInternal->freeing && !GET_MOTION_FREE_FLAG())
	{
		/* 只有所有轴都到位时才允许切换 */
		if (GET_MOTION_INPOS_FLAG())
		{
			rtapi_print_msg(RTAPI_MSG_DBG, "set free\n");
			for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++)
			{
				axis = &axes[axis_num];
				/* 同步自由规划器的当前位置为当前关节位置 */
				axis->free_tp.curr_pos = axis->pos_cmd;
				/* 保持自由规划器禁用，直到有实际的运动请求（点动命令）*/
				axis->free_tp.enable = 0;
			}
			SET_MOTION_FREE_FLAG(1);  /* 设置自由模式标志 */
			SET_MOTION_ERROR_FLAG(0);
		} else {
			/* 轴尚未到位，拒绝模式切换 */
			emcmotInternal->freeing = 0;
		}
	}

    /* 【阶段五】根据使能标志和模式标志，确定最终的 motion_state
     * 这是 motion_state 的最终更新点，之前所有阶段都只是设置了中间标志。
     * 实际的 motion_state 由以下逻辑决定：
     *   - 如果运动未使能 → DISABLED
     *   - 如果运动已使能且在 Coord 模式 → COORD
     *   - 如果运动已使能且不在 Coord 模式 → FREE
     * 注意：GET_MOTION_COORD_FLAG() 和 GET_MOTION_FREE_FLAG() 互斥，
     * 因此第三个分支在使能状态下必然是 FREE 模式。 */
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

/*
 * get_pos_cmds — 生成所有轴的位置/速度/加速度命令
 */
static void get_pos_cmds(long period)
{
    int axis_num, result;
    emcmot_axis_t *axis;// 轴结构体指针
    double positions[EMCMOT_MAX_AXIS]; // 笛卡尔/关节位置数组（临时存储）
    double vel_lim; // 速度限制

    /* used in teleop mode to compute the max accell requested */
    int onlimit = 0;  /* 软限位触发标志：任一轴超出软限位时为 1 */
    int joint_limit[EMCMOT_MAX_AXIS][2];  /* 关节限位状态数组
     * [n][0]: 负向限位触发标志
     * [n][1]: 正向限位触发标志 */

	/* 从坐标轨迹规划器中读取当前的笛卡尔位置（各轴的目标位置）
	 * 直接赋值，与上面注释掉的循环效果相同 */
	positions[0] = (int32_t)emcmotInternal->coord_tp.currentPos.tran.x;
	positions[1] = (int32_t)emcmotInternal->coord_tp.currentPos.tran.y;
	positions[2] = (int32_t)emcmotInternal->coord_tp.currentPos.tran.z;

    /* 根据当前运动状态（Free/Coord/Disabled）执行对应的运动计算逻辑 */
    switch ( emcmotStatus->motion_state)
    {
		    case EMCMOT_MOTION_FREE:
				/* 预设所有轴都处于到位状态。
				 * 如果后续发现某个规划器仍在运动，则清除 INPOS 标志。 */
				SET_MOTION_INPOS_FLAG(1);
				/* 遍历所有轴，对每个活跃轴执行独立的轨迹规划 */
				for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++)
				{
				    axis = &axes[axis_num];
				    /* 跳过非活跃轴 */
				    if (!GET_AXIS_ACTIVE_FLAG(axis))
				    {
				        continue;
				    }

				    /* 速度限幅：取轴配置和全局配置中较小的加速度值 */
				    if(axis->acc_limit > emcmotStatus->Maxacc)
						axis->acc_limit = emcmotStatus->Maxacc;  // 加速度限制
				    /* 【代码问题】这个分支的条件是 "motion_state != FREE"，
				     * 但此时 motion_state 正是 FREE，所以这个分支永远不执行。
				     * 这意味着 net_feed_scale 在 Free 模式下不会被应用。 */
					if ( emcmotStatus->motion_state != EMCMOT_MOTION_FREE )
					{
						vel_lim = axis->vel_limit * emcmotStatus->net_feed_scale;
						if (vel_lim > axis->vel_limit) {
							vel_lim = axis->vel_limit;  // 速度限制
						}

						if (vel_lim < axis->free_tp.max_vel)
							axis->free_tp.max_vel = vel_lim;
					}

					/* 设置自由轨迹规划器的最大加速度 */
					axis->free_tp.max_acc = axis->acc_limit;

					simple_tp_update(&(axis->free_tp), servo_period );

					/* 将计算出的位置写入原子变量，供外部组件读取 */
					atomic_store(&atomic_actpos[axis_num], axis->free_tp.curr_pos);

					/* 将规划器的输出写入轴的命令变量 */
					axis->pos_cmd = axis->free_tp.curr_pos;  // 设置位置命令
					axis->vel_cmd = axis->free_tp.curr_vel;  // 设置速度命令
					axis->acc_cmd = 0;  // 加速度命令在 Free 模式中不使用
					axis->coarse_pos = axis->free_tp.curr_pos; // 更新粗位置

					//如果规划器仍在工作设置不在位
					/* 检查规划器是否仍在运动（active == true 表示尚未到达目标）*/
					if ( axis->free_tp.active ) {
						SET_AXIS_INPOS_FLAG(axis, 0);  /* 标记该轴不在位 */
						SET_MOTION_INPOS_FLAG(0);     /* 标记全局不在位 */
						/* 如果有限位覆盖标志被设置，标记为 overriding 状态 */
						if ( emcmotStatus->overrideLimitMask )
						{
				           emcmotInternal->overriding = 1;
						}
					}
					else
					{
						SET_AXIS_INPOS_FLAG(axis, 1);  /* 该轴已到位 */
					}
				}

				//如果关掉限位并且运动到位，开启限位
    			/* 当限位覆盖模式下所有轴都回到安全位置后，
    			 * 自动清除限位覆盖标志和 overriding 状态 */
    			if ( (emcmotInternal->overriding ) && ( GET_MOTION_INPOS_FLAG() ) ) {
    				emcmotStatus->overrideLimitMask = 0;  /* 清除限位覆盖 */
    				emcmotInternal->overriding = 0;       /* 清除 overriding 状态 */
    			}
    			break;

    		case EMCMOT_MOTION_COORD:
    			//停止所有增量运动
    			/* 进入 Coord 模式后，自动中止所有手动点动运动。
    			 * 参数 1 表示急停模式（立即归零速度）。 */
    			axis_jog_abort_all(1);

    			//启动三次插补
    			/* 标记三次样条插值器处于激活状态 */
    			coord_cubic_active = 1;
    			//当插补器需要下一个点时，运行协调轨迹规划周期
    			// rtapi_print_msg(RTAPI_MSG_DBG, "axes[0].cubic.needNextPoint %d\n", axes[0].cubic.needNextPoint);
    			/* 运行 Coord 模式的轨迹规划循环。
    			 * 每次 cubicNeedNextPoint() 返回 true 时，表示插值器需要下一个数据点，
    			 * 此时运行一次 coord_tp 的规划周期来提供新的目标位置。 */
    			while (cubicNeedNextPoint(&(axes[0].cubic)))
    			{
    				/* 运行坐标轨迹规划器的一个伺服周期，
    				 * 计算到下一个目标位置的运动 */
    				tpRunCycle(&emcmotInternal->coord_tp, period);
    				// 获取笛卡尔位置
    				/* 获取 coord_tp 当前的笛卡尔目标位置 */
    				tpGetPos(&emcmotInternal->coord_tp, &emcmotStatus->carte_pos_cmd);

    				/* 更新位置数组，为后续处理做准备 */
    				positions[0] = emcmotStatus->carte_pos_cmd.tran.x;
    				positions[1] = emcmotStatus->carte_pos_cmd.tran.y;
    				positions[2] = emcmotStatus->carte_pos_cmd.tran.z;

					//更新坐标并检查边界
    				/* 应用外部偏移并检查软限位越界。
    				 * 如果越限，ext_offset_coord_limit 被设置为 1。 */
    				if (axis_update_coord_with_bound(pcmd_p, servo_period)) {
    					ext_offset_coord_limit = 1;
    				} else {
    					ext_offset_coord_limit = 0;
    				}

    				/* 遍历所有轴，为三次样条插值器提供新的目标点 */
    				for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++)
    				{
    					axis = &axes[axis_num];
    					if (!GET_AXIS_ACTIVE_FLAG(axis)) continue;

    					axis = &axes[axis_num];
    					// 设置粗位置
    					axis->coarse_pos = positions[axis_num];
    					// 添加点到三次插补器
    					/* 向三次样条插值器添加一个新的目标点。
    					 * 插值器会根据这些点计算平滑的关节位置曲线。 */
    					cubicAddPoint(&(axis->cubic), axis->coarse_pos);
    				}
				}

				/* 遍历所有轴，通过三次样条插值器获取平滑的位置/速度/加速度命令 */
				for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS;  axis_num++)
				{
					axis = &axes[axis_num];
					if (!GET_AXIS_ACTIVE_FLAG(axis)) continue;

				    axis = &axes[axis_num];
					// 三次插补获取位置、速度和加速度
					/* cubicInterpolate() 在给定时间点（此处为 0）处，
					 * 根据三次样条曲线计算当前位置、速度和加速度。
					 * vel_cmd 和 acc_cmd 通过输出参数返回。 */
					double vel_cmd = 0.0;
					double acc_cmd = 0.0;
				    axis->pos_cmd = cubicInterpolate(&(axis->cubic), 0, &vel_cmd, &acc_cmd, 0);

					axis->vel_cmd = vel_cmd;
					axis->acc_cmd = acc_cmd;
					/* 将计算出的位置写入原子变量 */
					atomic_store(&atomic_actpos[axis_num], axis->pos_cmd);

					/* 【性能问题】这些 rtapi_print_msg 每周期都执行，
					 * 在生产环境中会产生大量日志输出，应该注释掉或加上条件判断。 */
					rtapi_print_msg(RTAPI_MSG_DBG, "axis %d pos_cmd %d", axis_num, axis->pos_cmd);
					rtapi_print_msg(RTAPI_MSG_DBG, " vel_cmd %d\n", axis->vel_cmd);
				}

				/* 预设不在位，后续根据轨迹完成状态更新 */
				SET_MOTION_INPOS_FLAG(0);
    			// 如果轨迹规划完成，设置到位标志
				/* tpIsDone() 检查坐标轨迹规划器是否已完成所有排队的轨迹段。
				 * 如果完成了，设置全局到位标志。 */
				if (tpIsDone(&emcmotInternal->coord_tp)) {
				    SET_MOTION_INPOS_FLAG(1);
				}
				break;

		    case EMCMOT_MOTION_DISABLED:
				//指令位置等于反馈位置
				/* 当运动被禁用时，将笛卡尔位置命令同步到反馈位置。
				 * 这确保了禁用状态下没有跟随误差，轴保持静止。 */
				emcmotStatus->carte_pos_cmd = emcmotStatus->carte_pos_fb;
				/* 遍历所有轴，同步位置命令到实际反馈位置 */
				for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++)
				{
				    axis = &axes[axis_num];
				    axis->pos_cmd = axis->pos_fb;  /* 位置命令 = 反馈位置 */
				    axis->vel_cmd = 0.0;  /* 速度命令归零 */
				    axis->acc_cmd = 0.0;  /* 加速度命令归零 */
				}

				break;
		    default:
				break;
    }

	//检查软限位
    /* 软限位检查：在所有运动计算完成后，检查位置命令是否超出软限位范围。
     * 这是最后的安全检查，防止规划器产生的命令超出安全范围。 */
    for (axis_num = 0; axis_num < ALL_AXES; axis_num++)
    {
		/* point to joint data */
		axis = &axes[axis_num];

		/* 初始化限位数组为 0（无越限）*/
		joint_limit[axis_num][0] = 0;
		joint_limit[axis_num][1] = 0;

		/* 允许 1e-12 的容差来处理浮点精度误差。
		 * 超出正向限位：joint_limit[n][1] = 1
		 * 超出负向限位：joint_limit[n][0] = 1 */
		if (axis->pos_cmd > axis->max_pos_limit + 0.000000000001)
		{
		    joint_limit[axis_num][1] = 1;  /* 正向越限 */
            onlimit = 1;  /* 标记有越限发生 */
	    }
	    else if (axis->pos_cmd < axis->min_pos_limit - 0.000000000001)
	    {
		    joint_limit[axis_num][0] = 1;  /* 负向越限 */
            onlimit = 1;  /* 标记有越限发生 */
	    }
    }

    if ( onlimit ) //如果有软限位触发
    {
		/* 仅当限位状态发生变化（!on_soft_limit → on_soft_limit）时才打印错误消息 */
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
		    SET_MOTION_ERROR_FLAG(1);  /* 设置运动错误标志 */
		    emcmotStatus->on_soft_limit = 1;  /* 标记已进入软限位状态 */
		}
    } else {
		emcmotStatus->on_soft_limit = 0;  /* 清除软限位触发状态 */
    }
} // get_pos_cmds()

/*
 * output_to_hal — 将运动计算结果输出到 HAL 引脚
 */
static void output_to_hal(void)
{
	int axis_num;
	emcmot_axis_t *axis;   /* 指向轴内部运行时数据的指针 */
	axis_hal_t *axis_data; /* 指向轴的 HAL 数据的指针 */

	/* 输出全局运动状态到 HAL 引脚，供上位机和运动学模块读取 */
	*(emcmot_hal_data->motion_enabled) = GET_MOTION_ENABLE_FLAG();  /* 运动使能状态 */
	*(emcmot_hal_data->in_position) = GET_MOTION_INPOS_FLAG();     /* 全局到位状态 */
	*(emcmot_hal_data->coord_mode) = GET_MOTION_COORD_FLAG();     /* 坐标联动模式 */
	*(emcmot_hal_data->coord_error) = GET_MOTION_ERROR_FLAG();   /* 运动错误 */
	*(emcmot_hal_data->on_soft_limit) = emcmotStatus->on_soft_limit;  /* 软限位触发 */

	/* 遍历所有轴，输出各轴的数据到 HAL 引脚 */
	for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++)
	{
		/* point to joint struct */
		axis = &axes[axis_num];
		axis_data = &(emcmot_hal_data->axis[axis_num]);

		/* 跳过非活跃轴 */
		if(!GET_AXIS_ACTIVE_FLAG(axis))
		{
			continue;
		}

		/* apply backlash and motor offset to output
		 * 计算电机位置命令：将关节位置命令加上电机偏移量
		 * motor_offset 在回零过程中建立，用于补偿电机位置到机床坐标系的映射。
		 * 最终的 motor_pos_cmd 才是发送给伺服驱动器的实际位置命令。 */
		axis->motor_pos_cmd = axis->pos_cmd + axis->motor_offset;

		// //更新电机位置
		/* 将电机位置命令写入原子变量，供外部组件（如下位机通信）原子读取 */
		atomic_store(&atomic_targetpos[axis_num], axis->motor_pos_cmd);

		/* 以下是将各轴数据写入对应的 HAL 引脚，供伺服驱动器和上位机使用 */
		*(axis_data->motor_offset) = axis->motor_offset;            //电机偏移量
		*(axis_data->motor_pos_cmd) = axis->motor_pos_cmd;           //电机位置命令（含补偿）
		*(axis_data->axis_pos_cmd) = axis->pos_cmd;                 //原始轴位置命令（不含补偿）
		*(axis_data->axis_pos_fb) = axis->pos_fb;                   //轴位置反馈
		*(axis_data->amp_enable) = GET_AXIS_ENABLE_FLAG(axis);      //使能状态
		*(axis_data->coarse_pos_cmd) = axis->coarse_pos;            //粗位置命令（Coord 模式）
		*(axis_data->axis_vel_cmd) = axis->vel_cmd;                  //轴速度命令
		*(axis_data->axis_acc_cmd) = axis->acc_cmd;                  //轴加速度命令
		*(axis_data->f_error) = axis->ferror;                        //跟随误差
		*(axis_data->f_error_lim) = axis->ferror_limit;              //跟随误差限制值

		/* 以下是轴的状态标志引脚，反映轴的当前运行状态 */
		*(axis_data->active) = GET_AXIS_ACTIVE_FLAG(axis);           //轴激活状态
		*(axis_data->motionenable) = emcmotInternal->enabling;      //运动使能状态
		*(axis_data->motionrunning) = axis->free_tp.enable;        //规划器运行状态
		*(axis_data->in_position) = GET_AXIS_INPOS_FLAG(axis);       //轴在位状态（是否到达目标位置）
		*(axis_data->error) = GET_AXIS_ERROR_FLAG(axis);             //轴错误状态
		*(axis_data->phl) = GET_AXIS_PHL_FLAG(axis);                //正硬限位状态
		*(axis_data->nhl) = GET_AXIS_NHL_FLAG(axis);                //负硬限位状态
		*(axis_data->f_errored) = GET_AXIS_FERROR_FLAG(axis);      //跟随错误状态
		*(axis_data->faulted) = GET_AXIS_FAULT_FLAG(axis);          //输出故障状态

    }

	/* 调用 axis_output_to_hal()，输出遥操作相关的数据到 HAL*/
	axis_output_to_hal(pcmd_p);

	/* 更新 HAL 引脚：当前是否有活跃的点动运动 */
	*(emcmot_hal_data->jog_is_active) = axis_jog_is_active();
}

/*
 * update_status — 更新状态结构体供 Task 层查询
 */
static void update_status(void)
{
    int axis_num, misc_error;
    emcmot_axis_t *axis;        /* 指向轴内部运行时数据的指针 */
    emcmot_axis_status_t *axis_status;  /* 指向轴状态结构体的指针 */

    /* 将各轴的运行时状态复制到 emcmotStatus 的 axis_status[] 数组中，
     * 供 Task 层通过 NML 缓冲区读取。 */
    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++) {
	axis = &axes[axis_num];
	axis_status = &(emcmotStatus->axis_status[axis_num]);

	/* 依次复制各轴的关键状态字段 */
	axis_status->flag = axis->flag;                   /* 轴状态标志 */
	axis_status->pos_cmd = axis->pos_cmd;             /* 位置命令 */
	axis_status->pos_fb = axis->pos_fb;               /* 位置反馈 */
	axis_status->vel_cmd = axis->vel_cmd;            /* 速度命令 */
	axis_status->acc_cmd = axis->acc_cmd;             /* 加速度命令 */
	axis_status->ferror = axis->ferror;               /* 跟随误差 */
	axis_status->ferror_high_mark = axis->ferror_high_mark;  /* 最大跟随误差 */
	axis_status->max_pos_limit = axis->max_pos_limit;      /* 最大软限位 */
	axis_status->min_pos_limit = axis->min_pos_limit;        /* 最小软限位 */
	axis_status->min_ferror = axis->min_ferror;              /* 最小跟随误差限值 */
	axis_status->max_ferror = axis->max_ferror;              /* 最大跟随误差限值 */
    }


    /* 再次遍历所有轴，通过 axis.c 中的 getter 函数读取一些只读数据。
     * 这些数据来自 axis_array[]（axis.c 私有），可能包含一些 axes[] 中没有的字段。 */
    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++) {
        /* point to axis status */
        axis_status = &(emcmotStatus->axis_status[axis_num]);

        /* 读取遥操作速度命令、软限位等只读数据 */
        axis_status->axis_vel_cmd = axis_get_teleop_vel_cmd(axis_num);
        axis_status->max_pos_limit = axis_get_max_pos_limit(axis_num);
        axis_status->min_pos_limit = axis_get_min_pos_limit(axis_num);
    }
    /* 更新外部偏移的笛卡尔坐标 */
    emcmotStatus->eoffset_pose.tran.x = axis_get_ext_offset_curr_pos(0);
    emcmotStatus->eoffset_pose.tran.y = axis_get_ext_offset_curr_pos(1);
    emcmotStatus->eoffset_pose.tran.z = axis_get_ext_offset_curr_pos(2);
    // emcmotStatus->eoffset_pose.a      = axis_get_ext_offset_curr_pos(3);
    // emcmotStatus->eoffset_pose.b      = axis_get_ext_offset_curr_pos(4);
    // emcmotStatus->eoffset_pose.c      = axis_get_ext_offset_curr_pos(5);
    // emcmotStatus->eoffset_pose.u      = axis_get_ext_offset_curr_pos(6);
    // emcmotStatus->eoffset_pose.v      = axis_get_ext_offset_curr_pos(7);
    // emcmotStatus->eoffset_pose.w      = axis_get_ext_offset_curr_pos(8);

    /* 更新外部偏移活动状态（从 HAL 引脚读取）*/
    emcmotStatus->external_offsets_applied = *(emcmot_hal_data->eoffset_active);

    /* 复制杂项错误数组（从 HAL 引脚到状态结构体）*/
    for (misc_error=0; misc_error < emcmotConfig->numMiscError; misc_error++){
      emcmotStatus->misc_error[misc_error] = *(emcmot_hal_data->misc_error[misc_error]);
    }

    /* 更新点动活跃状态 */
    emcmotStatus->jogging_active = *(emcmot_hal_data->jog_is_active);

    /* 以下是从坐标轨迹规划器中读取的状态信息，供 Task 层查询。 */
    emcmotStatus->depth = tpQueueDepth(&emcmotInternal->coord_tp);        /* 轨迹队列深度 */
    emcmotStatus->activeDepth = tpActiveDepth(&emcmotInternal->coord_tp);  /* 活跃轨迹深度 */
    emcmotStatus->id = tpGetExecId(&emcmotInternal->coord_tp);              /* 当前执行轨迹段 ID */
    //KLUDGE add an API call for this
    emcmotStatus->reverse_run = emcmotInternal->coord_tp.reverse_run;     /* 反向运行标志 */

    emcmotStatus->motionType = tpGetMotionType(&emcmotInternal->coord_tp);  /* 当前运动类型 */
    emcmotStatus->queueFull = tcqFull(&emcmotInternal->coord_tp.queue);    /* 队列是否已满 */

    /* 单步执行模式：如果启用了单步执行（stepping == true），
     * 且当前执行的轨迹段 ID 与预期不同，则暂停等待。
     * 这用于实现"一次执行一个轨迹段"的调试模式。 */
    if (emcmotStatus->stepping && emcmotInternal->idForStep != emcmotStatus->id) {
      tpPause(&emcmotInternal->coord_tp);  /* 暂停轨迹规划 */
      emcmotStatus->stepping = 0;          /* 清除单步标志 */
      emcmotStatus->paused = 1;           /* 设置暂停标志 */
    }
}
