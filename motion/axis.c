#include "axis.h"
#include "emcmotcfg.h"
#include "rtapi/rtapi.h"
#include "rtapi/rtapi_math.h"
#include "simple_tp.h"
#include "hal/hal.h"

/*
 * emcmot_axis_t — 轴的内部运行时数据结构
 */
typedef struct {

    double pos_cmd;                 /* 轴命令位置 */
    double teleop_vel_cmd;          /* 轴命令速度 */
    double max_pos_limit;           /* 轴的正向软限位 */
    double min_pos_limit;           /* 轴的负向软限位 */
    double vel_limit;               /* 轴速度上限 */
    double acc_limit;               /* 轴加速度上限 */

    simple_tp_t teleop_tp;          /* 遥操作轨迹规划器 */

    int old_ajog_counts;            /* 上一次手轮脉冲计数 */
    int kb_ajog_active;             /* 键盘点动激活标志 */
    int locking_joint;              /* 锁死轴编号，-1表示未使用 */

    double ext_offset_vel_limit;    /* 外部偏移速度上限 */
    double ext_offset_acc_limit;    /* 外部偏移加速度上限 */
    int old_eoffset_counts;
    simple_tp_t ext_offset_tp;      /* 外部偏移轨迹规划器 */
} emcmot_axis_t;

/*
 * axis_hal_t — 轴的 HAL（硬件抽象层）引脚数据结构
 */
typedef struct {

    hal_float_t *pos_cmd;           /* 轴命令位置 */
    hal_float_t *teleop_vel_cmd;    /* 遥操作速度命令 */
    hal_float_t *teleop_pos_cmd;    /* 遥操作轨迹规划器的位置命令 */
    hal_float_t *teleop_vel_lim;    /* 遥操作规划器的速度限值 */
    hal_bit_t   *teleop_tp_enable;  /* 遥操作规划器运行标志 */
    hal_s32_t   *ajog_counts;       /* 手轮位置脉冲计数 */
    hal_bit_t   *ajog_enable;       /* 手轮使能信号 */
    hal_float_t *ajog_scale;        /* 每个脉冲对应的运动距离 */
    hal_float_t *ajog_accel_fraction;  /* 手轮点动加速度比例系数 */
    hal_bit_t   *ajog_vel_mode;     /* 速度模式手轮标志 */
    hal_bit_t   *kb_ajog_active;    /* 键盘点动执行标志 */
    hal_bit_t   *eoffset_enable;    /* 外部偏移功能使能信号 */
    hal_bit_t   *eoffset_clear;     /* 外部偏移清除信号 */
    hal_s32_t   *eoffset_counts;    /* 外部偏移专用手轮的脉冲计数 */
    hal_float_t *eoffset_scale;     /* 每个外部偏移脉冲对应的偏移量 */
    hal_float_t *external_offset;   /* 外部偏移的当前实际值 */
    hal_float_t *external_offset_requested; /* 用户请求的外部偏移目标值 */

} axis_hal_t;


typedef struct {
    axis_hal_t axis[EMCMOT_MAX_AXIS];   /* 每个轴的 HAL 数据 */
} axis_hal_data_t;

static emcmot_axis_t axis_array[EMCMOT_MAX_AXIS]; /* axis_array : 轴的内部运行时数据数组 */

static axis_hal_data_t *axis_hal_data = NULL; /* axis_hal_data : 指向所有轴 HAL 数据的指针 */

static int export_axis(int mot_comp_id, char c, axis_hal_t * addr);

/*
 * _() 宏 — 国际化字符串标记（i18n）
 */
#define _(s) (s)

/*
 * CALL_CHECK() 宏 — 错误检查与短路返回
 */
#define CALL_CHECK(expr) do {           \
        int _retval;                    \
        _retval = expr;                 \
        if (_retval) return _retval;    \
    } while (0);

/*
 * axis_init_all — 初始化所有轴的默认状态
 */
void axis_init_all(void)
{
    int n;
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        /* 获取当前轴的数据结构指针（引用） */
        emcmot_axis_t *axis = &axis_array[n];
        /* 将锁死轴编号初始化为 -1，表示当前轴没有关联锁死机构 */
        axis->locking_joint = -1;
    }
}


/*
 * axis_initialize_external_offsets — 初始化所有轴的外部偏移状态
 */
void axis_initialize_external_offsets(void)
{
    int n;
    axis_hal_t *axis_data;

    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        /* 获取当前轴的 HAL 数据结构指针 */
        axis_data = &axis_hal_data->axis[n];

        /* 重置 HAL 引脚：external_offset 和 external_offset_requested 均归零 */
        *(axis_data->external_offset) = 0;
        *(axis_data->external_offset_requested) = 0;

        /* 重置外部偏移轨迹规划器（ext_offset_tp）的状态 */
        /* pos_cmd = 0 : 目标偏移量归零（即没有请求任何偏移） */
        axis_array[n].ext_offset_tp.pos_cmd  = 0;
        /* curr_pos = 0 : 当前位置归零（规划器从零开始规划） */
        axis_array[n].ext_offset_tp.curr_pos = 0;
        /* curr_vel = 0 : 当前速度归零（静止状态） */
        axis_array[n].ext_offset_tp.curr_vel = 0;
    }
}

/*
 * export_axis — 导出单个轴的手轮相关 HAL 引脚
 */
static int export_axis(int mot_comp_id, char c, axis_hal_t * addr)
{
    hal_pin_newf(HAL_BIT,  &(addr->ajog_enable), mot_comp_id,"axis.%c.jog-enable", c);
    hal_pin_newf(HAL_FLOAT, &(addr->ajog_scale), mot_comp_id,"axis.%c.jog-scale", c);
    hal_pin_newf(HAL_U32, &(addr->ajog_counts), mot_comp_id,"axis.%c.jog-counts", c);
    hal_pin_newf(HAL_BIT,  &(addr->ajog_vel_mode), mot_comp_id,"axis.%c.jog-vel-mode", c);
    hal_pin_newf(HAL_BIT,  &(addr->kb_ajog_active), mot_comp_id,"axis.%c.kb-jog-active", c);
    hal_pin_newf(HAL_FLOAT, &(addr->ajog_accel_fraction), mot_comp_id,"axis.%c.jog-accel-fraction", c);
    *addr->ajog_accel_fraction = 1.0; // fraction of accel for wheel ajogs

    return 0;
}


/*
 * axis_init_hal_param — 初始化所有轴的 HAL 参数并导出引脚
 */
int axis_init_hal_param(int mot_comp_id)
{
    int n, retval;

    /* 在 HAL 共享内存中分配 axis_hal_data_t 结构体 */
    axis_hal_data = hal_malloc(sizeof(axis_hal_data_t));
    /* 检查分配是否成功 */
    if (!axis_hal_data) {
        /* hal_malloc 失败通常是 HAL 共享内存区域耗尽，打印错误并返回 */
        rtapi_print_msg(RTAPI_MSG_ERR, "MOTION: axis_hal_data hal_malloc() failed\n");
        return -1;
    }

    /* 遍历所有轴，为每个轴创建 HAL 引脚 */
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        /* 根据轴编号 n 获取对应的轴名字符
         * 索引对应关系：0→'x', 1→'y', 2→'z', 3→'a', 4→'b', 5→'c', 6→'u', 7→'v', 8→'w'
         * 这是 LinuxCNC 中标准的轴命名约定 */
        char c = "xyzabcuvw"[n];
        /* 获取当前轴的 axis_hal_t 数据结构指针 */
        axis_hal_t *axis_data = &(axis_hal_data->axis[n]);

        /* 创建位置命令引脚（RPI）：供其他组件读取轴的位置命令 */
        hal_pin_newf(HAL_FLOAT, &axis_data->pos_cmd, mot_comp_id, "axis.%c.pos-cmd", c);
        /* 创建遥操作速度命令引脚（RPI） */
        hal_pin_newf(HAL_FLOAT, &axis_data->teleop_vel_cmd, mot_comp_id, "axis.%c.teleop-vel-cmd", c);
        /* 创建遥操作位置命令引脚（RPI） */
        hal_pin_newf(HAL_FLOAT, &axis_data->teleop_pos_cmd, mot_comp_id, "axis.%c.teleop-pos-cmd", c);
        /* 创建遥操作速度限值引脚（RPI） */
        hal_pin_newf(HAL_FLOAT, &axis_data->teleop_vel_lim, mot_comp_id, "axis.%c.teleop-vel-lim", c);
        /* 创建遥操作规划器使能引脚（RPI） */
        hal_pin_newf(HAL_BIT,  &axis_data->teleop_tp_enable, mot_comp_id, "axis.%c.teleop-tp-enable",c);
        /* 创建外部偏移使能引脚（RPI/WPI） */
        hal_pin_newf(HAL_BIT, &axis_data->eoffset_enable, mot_comp_id, "axis.%c.eoffset-enable", c);
        /* 创建外部偏移清除引脚（RPI/WPI） */
        hal_pin_newf(HAL_BIT, &axis_data->eoffset_clear, mot_comp_id, "axis.%c.eoffset-clear", c);
        /* 创建外部偏移脉冲计数引脚（WPI） */
        hal_pin_newf(HAL_U32,  &axis_data->eoffset_counts, mot_comp_id, "axis.%c.eoffset-counts", c);
        /* 创建外部偏移比例系数引脚（RPI） */
        hal_pin_newf(HAL_FLOAT, &axis_data->eoffset_scale, mot_comp_id, "axis.%c.eoffset-scale", c);
        /* 创建外部偏移当前值引脚（RPI） */
        hal_pin_newf(HAL_FLOAT,  &axis_data->external_offset, mot_comp_id, "axis.%c.eoffset", c);
        /* 创建外部偏移请求值引脚（RPI） */
        hal_pin_newf(HAL_FLOAT,  &axis_data->external_offset_requested,
           mot_comp_id, "axis.%c.eoffset-request", c);

        /* 调用 export_axis() 导出当前轴的手轮相关引脚
         * 该函数负责导出 jog-enable, jog-scale, jog-counts, jog-vel-mode,
         * kb-jog-active, jog-accel-fraction 等 */
        retval = export_axis(mot_comp_id, c, axis_data);
        if (retval)
        {
            /* 引脚导出失败，打印错误消息并返回
             * _() 宏将字符串标记为待翻译文本（在用户空间层生效） */
            rtapi_print_msg(RTAPI_MSG_ERR, _("MOTION: axis %c pin/param export failed\n"), c);
            return -1;
        }
    }
}

/*
 * axis_output_to_hal — 将轴状态输出到 HAL 引脚
 */
void axis_output_to_hal(double *pcmd_p[])
{
    int n;

    for (n = 0; n < EMCMOT_MAX_AXIS; n++)
    {
        /* 获取当前轴的内部数据和 HAL 数据结构 */
        emcmot_axis_t *axis = &axis_array[n];
        axis_hal_t *axis_data = &axis_hal_data->axis[n];

        /* 将遥操作速度命令写入 HAL */
        *(axis_data->teleop_vel_cmd)    = axis->teleop_vel_cmd;
        /* 将遥操作位置命令写入 HAL（规划器的目标位置） */
        *(axis_data->teleop_pos_cmd)    = axis->teleop_tp.pos_cmd;
        /* 将遥操作速度限值写入 HAL */
        *(axis_data->teleop_vel_lim)    = axis->teleop_tp.max_vel;
        /* 将遥操作规划器使能状态写入 HAL */
        *(axis_data->teleop_tp_enable)  = axis->teleop_tp.enable;
        /* 将键盘点动激活状态写入 HAL */
        *(axis_data->kb_ajog_active)    = axis->kb_ajog_active;
        /* 写入位置命令  */
        *(axis_data->pos_cmd) = *pcmd_p[n]- axis->ext_offset_tp.curr_pos;
    }
}

/* 设置轴的正向软限位。当轴的位置命令超过此值时，系统将触发限位保护。 */
void axis_set_max_pos_limit(int axis_num, double maxLimit)
{
    axis_array[axis_num].max_pos_limit = maxLimit;
}

/* 设置轴的负向软限位。当轴的位置命令低于此值时，系统将触发限位保护。 */
void axis_set_min_pos_limit(int axis_num, double minLimit)
{
    axis_array[axis_num].min_pos_limit = minLimit;
}

/* 设置轴的最大速度限制。所有运动指令的速度都不能超过此值。 */
void axis_set_vel_limit(int axis_num, double vel)
{
    axis_array[axis_num].vel_limit = vel;
}

/* 设置轴的最大加速度限制。所有运动指令的加速度都不能超过此值。 */
void axis_set_acc_limit(int axis_num, double acc)
{
    axis_array[axis_num].acc_limit = acc;
}

/* 设置外部偏移运动的最大速度限制（仅影响 ext_offset_tp 规划器）。 */
void axis_set_ext_offset_vel_limit(int axis_num, double vel)
{
    axis_array[axis_num].ext_offset_vel_limit = vel;
}

/* 设置外部偏移运动的最大加速度限制（仅影响 ext_offset_tp 规划器）。 */
void axis_set_ext_offset_acc_limit(int axis_num, double acc)
{
    axis_array[axis_num].ext_offset_acc_limit = acc;
}

/* 设置轴关联的锁死轴编号。= -1 表示该轴不关联任何锁死机构。 */
void axis_set_locking_joint(int axis_num, int joint)
{
    axis_array[axis_num].locking_joint = joint;
}

/* 返回指定轴的负向（最小）软限位值。 */
double axis_get_min_pos_limit(int axis_num)
{
    return axis_array[axis_num].min_pos_limit;
}

/* 返回指定轴的正向（最大）软限位值。 */
double axis_get_max_pos_limit(int axis_num)
{
    return axis_array[axis_num].max_pos_limit;
}

/* 返回指定轴的最大速度限制（单位：用户单位/秒）。 */
double axis_get_vel_limit(int axis_num)
{
    return axis_array[axis_num].vel_limit;
}

/* 返回指定轴的最大加速度限制（单位：用户单位/秒²）。 */
double axis_get_acc_limit(int axis_num)
{
    return axis_array[axis_num].acc_limit;
}

/* 返回指定轴的遥操作速度命令（当前正在执行的速度值）。 */
double axis_get_teleop_vel_cmd(int axis_num)
{
    return axis_array[axis_num].teleop_vel_cmd;
}

/* 返回指定轴关联的锁死轴编号（-1 表示无锁死轴）。 */
int axis_get_locking_joint(int axis_num)
{
    return axis_array[axis_num].locking_joint;
}

/*
 * axis_get_compound_velocity — 计算所有激活轴的合成速度
 */
double axis_get_compound_velocity(void)
{
    double v2 = 0.0;  /* 累加各轴速度的平方和 */
    int n;

    /* 遍历所有轴，累加激活轴速度的平方 */
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        emcmot_axis_t *axis = &axis_array[n];
        /* 仅累加正在运动（active == true）的轴 */
        if (axis->teleop_tp.active) {
            v2 += axis->teleop_vel_cmd * axis->teleop_vel_cmd;
        }
    }

    /* 如果有轴在运动，计算平方和的平方根（欧几里得范数） */
    if (v2 > 0.0)
        return sqrt(v2);
    return 0.0;
}

/* 返回指定轴的外部偏移轨迹规划器的当前位置（当前偏移量）。 */
double axis_get_ext_offset_curr_pos(int axis_num)
{
    return axis_array[axis_num].ext_offset_tp.curr_pos;
}

/*
 * axis_jog_cont — 连续点动（Continuous Jog）
 */
void axis_jog_cont(int axis_num, double vel, long servo_period)
{
    (void)servo_period;  /* servo_period 参数本函数未使用，消除编译器警告 */
    emcmot_axis_t *axis = &axis_array[axis_num];

    /* 根据速度方向设置目标位置为对应方向的软限位 */
    if (vel > 0.0) {
        /* 正向运动：将目标位置设置为正向软限位 */
        axis->teleop_tp.pos_cmd = axis->max_pos_limit;
    } else {
        /* 负向运动：将目标位置设置为负向软限位 */
        axis->teleop_tp.pos_cmd = axis->min_pos_limit;
    }

    /* 配置轨迹规划器的速度与加速度参数 */
    /* max_vel 取绝对值（fabs），确保速度为正 */
    axis->teleop_tp.max_vel = fabs(vel);
    /* max_acc 使用轴的最大加速度限制 */
    axis->teleop_tp.max_acc = axis->acc_limit;
    /* 标记键盘点动处于激活状态，防止与手轮点动冲突 */
    axis->kb_ajog_active = 1;
    /* 使能轨迹规划器，开始运动规划 */
    axis->teleop_tp.enable = 1;
}


/*
 * axis_jog_incr — 增量点动（Incremental Jog）
 */
void axis_jog_incr(int axis_num, double offset, double vel, long servo_period)
{
    (void)servo_period;  /* 未使用，抑制警告 */
    emcmot_axis_t *axis = &axis_array[axis_num];
    double tmp1;  /* 临时变量：计算后的新目标位置 */

    /* 根据速度方向（vel 的符号）决定是加还是减 offset */
    if (vel > 0.0) {
        tmp1 = axis->teleop_tp.pos_cmd + offset;
    } else {
        tmp1 = axis->teleop_tp.pos_cmd - offset;
    }

    /* 软限位越界检查：如果计算出的目标位置超出软限位范围，则直接返回 */
    if (tmp1 > axis->max_pos_limit) { return; }
    if (tmp1 < axis->min_pos_limit) { return; }

    /* 通过限位检查，设置规划器的目标位置、速度、加速度并使能 */
    axis->teleop_tp.pos_cmd = tmp1;
    axis->teleop_tp.max_vel = vel;
    axis->teleop_tp.max_acc = axis->acc_limit;
    axis->kb_ajog_active = 1;
    axis->teleop_tp.enable = 1;
}


/*
 * axis_jog_abs — 绝对点动（Absolute Jog）
 */
void axis_jog_abs(int axis_num, double offset, double vel)
{
    emcmot_axis_t *axis = &axis_array[axis_num];
    double tmp1;

    /* 首先标记键盘点动处于激活状态 */
    axis->kb_ajog_active = 1;
    /* 根据速度方向计算目标位置（注意：此实现与函数名"绝对点动"不符，
     * 实际上仍然是基于当前位置的增量计算） */
    if (vel > 0.0) {
        tmp1 = axis->teleop_tp.pos_cmd + offset;
    } else {
        tmp1 = axis->teleop_tp.pos_cmd - offset;
    }
    /* 限位检查：如果目标位置越界，直接返回 */
    if (tmp1 > axis->max_pos_limit) { return; }
    if (tmp1 < axis->min_pos_limit) { return; }
    /* 设置规划器参数并使能 */
    axis->teleop_tp.pos_cmd = tmp1;
    axis->teleop_tp.max_vel = vel;
    axis->teleop_tp.max_acc = axis->acc_limit;
    axis->kb_ajog_active = 1;
    axis->teleop_tp.enable = 1;
}


/*
 * axis_jog_abort — 中止指定轴的点动操作
 */
bool axis_jog_abort(int axis_num, bool immediate)
{
    bool aborted = 0;
    emcmot_axis_t *axis = &axis_array[axis_num];
    /* 检查当前是否有活跃的点动（规划器是否启用） */
    if (axis->teleop_tp.enable) {
        aborted = 1;  /* 标记确实中止了某个活跃的点动 */
    }
    /* 禁用轨迹规划器（无论是否真的在运动，都重置使能标志） */
    axis->teleop_tp.enable = 0;
    /* 清除键盘点动激活标志，允许其他点动方式接管 */
    axis->kb_ajog_active = 0;
    /* 如果请求立即停止，则将当前速度强制清零（急停） */
    if (immediate) {
        axis->teleop_tp.curr_vel = 0.0;
    }
    return aborted;  /* 返回是否有实际中止操作发生 */
}


/*
 * axis_jog_abort_all — 中止所有轴的点动操作
 */
bool axis_jog_abort_all(bool immediate)
{
    int n;
    bool aborted = 0;
    /* 遍历所有轴，依次中止每个轴的点动 */
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        if (axis_jog_abort(n, immediate)) {aborted = 1;}
    }
    return aborted;
}


/*
 * axis_jog_is_active — 查询是否有任何轴正在进行点动
 */
bool axis_jog_is_active(void)
{
    int n;
    emcmot_axis_t *axis;
    /* 遍历所有轴，查找是否有任何轴处于键盘点动状态 */
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        axis = &axis_array[n];
        /* 只要发现任何一个轴的 kb_ajog_active 为真，就返回 true */
        if (axis->kb_ajog_active) {
            return 1;
        }
    }
    return 0;
}

/*
 * axis_handle_jogwheels — 处理所有手轮的输入
 */
void axis_handle_jogwheels(bool motion_teleop_flag, bool motion_enable_flag, bool homing_is_active)
{
    int axis_num;
    emcmot_axis_t *axis;
    axis_hal_t *axis_data;
    int new_ajog_counts, delta;  /* delta: 本周期手轮脉冲增量（有方向） */
    double distance, pos, stop_dist;  /* distance: 对应轴运动量, pos: 目标位置, stop_dist: 安全停止距离 */
    static int first_pass = 1;	/* used to set initial conditions（首次调用标志，用于初始化 old_ajog_counts） */

    /* 遍历所有轴，依次处理每个轴的手轮输入 */
    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++) {
        double aaccel_limit;  /* 实际使用的加速度上限（可能小于 acc_limit） */
        axis = &axis_array[axis_num];
        axis_data = &axis_hal_data->axis[axis_num];

        // disallow accel bogus fractions
        /* 步骤 1：计算加速度限制
         * 如果用户配置的比例系数越界（<0 或 >1），则回退使用轴的最大加速度 */
        if (   (*(axis_data->ajog_accel_fraction) > 1)
            || (*(axis_data->ajog_accel_fraction) < 0) ) {
            aaccel_limit = axis->acc_limit;
        } else {
            aaccel_limit = *(axis_data->ajog_accel_fraction) * axis->acc_limit;
        }

        /* 步骤 2：读取当前手轮脉冲计数 */
        new_ajog_counts = *(axis_data->ajog_counts);
        /* 步骤 3：计算脉冲增量（本次计数值 - 上次计数值） */
        delta = new_ajog_counts - axis->old_ajog_counts;
        /* 步骤 3（续）：更新旧值，为下一个周期做准备 */
        axis->old_ajog_counts = new_ajog_counts;
        /* 步骤 4：首次调用时，仅更新计数，不产生运动命令 */
        if ( first_pass ) { continue; }
        /* 步骤 5：手轮没有转动，无需处理 */
        if ( delta == 0 ) {
            //just update counts
            continue;
        }
        /* 步骤 6：遥操作模式检查——如果不在遥操作模式，立即停止所有轴 */
        if (!motion_teleop_flag) {
            axis->teleop_tp.enable = 0;
            return;
        }
        /* 步骤 7：运动使能检查 */
        if (!motion_enable_flag)              { continue; }
        /* 步骤 8：手轮使能检查 */
        if ( *(axis_data->ajog_enable) == 0 ) { continue; }
        /* 步骤 9：回零状态检查——回零过程中禁用手轮 */
        if (homing_is_active)                 { continue; }
        /* 步骤 10：键盘点动冲突检查——键盘点动时忽略手轮 */
        if (axis->kb_ajog_active)             { continue; }

        /* 步骤 11：锁死轴检查——锁死轴不允许手轮点动 */
        if (axis->locking_joint >= 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
            "Cannot wheel jog a locking indexer AXIS_%c\n",
            "XYZABCUVW"[axis_num]);
            continue;
        }

        /* 步骤 12：计算手轮脉冲对应的轴运动量 */
        distance = delta * *(axis_data->ajog_scale);
        /* 步骤 13：计算新的目标位置 */
        pos = axis->teleop_tp.pos_cmd + distance;
        /* 步骤 14-15：速度模式下的安全距离限制 */
        if ( *(axis_data->ajog_vel_mode) ) {
            double v = axis->vel_limit;
            /* 计算以最大速度运动时的安全停止距离
             * 公式来源：v² = 2 * a * s => s = v² / (2 * a)
             * 这是在当前加速度限制下，从最大速度减速到零所需的最小距离 */
            stop_dist = v * v / ( 2 * aaccel_limit);
            /* 如果目标位置超出"当前位置 + 停止距离"，则截断目标位置
             * 这样可以防止手轮转得过快时，轴冲出现有位置（缺少刹车空间） */
            if ( pos > axis->pos_cmd + stop_dist ) {
                pos = axis->pos_cmd + stop_dist;
            } else if ( pos < axis->pos_cmd - stop_dist ) {
                pos = axis->pos_cmd - stop_dist;
            }
        }
        /* 步骤 16：软限位越界检查 */
        if (pos > axis->max_pos_limit) { break; }
        if (pos < axis->min_pos_limit) { break; }
        /* 步骤 17：将计算结果写入规划器，使能运动 */
        axis->teleop_tp.pos_cmd = pos;
        axis->teleop_tp.max_vel = axis->vel_limit;
        axis->teleop_tp.max_acc = aaccel_limit;
        axis->teleop_tp.enable  = 1;
    }
    /* 步骤 18：清除首次调用标志，表示初始化完成 */
    first_pass = 0;
}

/*
 * axis_sync_teleop_tp_to_carte_pos — 将遥操作规划器同步到笛卡尔位置
 */
void axis_sync_teleop_tp_to_carte_pos(int extfactor, double *pcmd_p[])
{
    int n;
    // expect extfactor =  -1 || 0 || +1
    /* 遍历所有轴，同步每个轴的遥操作规划器位置到笛卡尔位置数组 */
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        axis_array[n].teleop_tp.curr_pos = *pcmd_p[n]
                            + extfactor * axis_array[n].ext_offset_tp.curr_pos;
    }
}


/*
 * axis_sync_carte_pos_to_teleop_tp — 将笛卡尔位置同步到遥操作规划器
 */
void axis_sync_carte_pos_to_teleop_tp(int extfactor, double *pcmd_p[])
{
    int n;
    // expect extfactor =  -1 || 0 || +1
    /* 遍历所有轴，同步笛卡尔位置到遥操作规划器 */
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        *pcmd_p[n] = axis_array[n].teleop_tp.curr_pos
                            + extfactor * axis_array[n].ext_offset_tp.curr_pos;
    }
}


/*
 * axis_apply_ext_offsets_to_carte_pos — 将外部偏移应用到笛卡尔位置
 */
void axis_apply_ext_offsets_to_carte_pos(int extfactor, double *pcmd_p[])
{
    int n;
    // expect extfactor =  -1 || 0 || +1
    /* 遍历所有轴，将外部偏移叠加到笛卡尔位置上 */
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        *pcmd_p[n] = *pcmd_p[n] + extfactor * axis_array[n].ext_offset_tp.curr_pos;
    }
}

/*
 * axis_plan_external_offsets — 规划外部坐标偏移运动
 */
bool axis_plan_external_offsets(double servo_period, bool motion_enable_flag, bool all_homed)
{
    static int first_pass = 1;  /* 首次调用标志，用于跳过第一个周期的初始化处理 */
    int n;
    emcmot_axis_t *axis;
    axis_hal_t *axis_data;
    int new_eoffset_counts, delta;  /* 外部偏移脉冲计数的当前值和增量 */
    static int last_eoffset_enable[EMCMOT_MAX_AXIS];  /* 上周期各轴的使能状态（用于边沿检测） */
    double ext_offset_epsilon;  /* 静止判定阈值：当偏移量小于此值时认为轴已静止 */
    hal_bit_t eoffset_active;  /* 外部偏移活跃标志：任一轴的偏移量大于阈值时为 1 */

    eoffset_active = 0;  /* 初始化为非活跃 */

    /* 遍历所有轴，依次处理每个轴的外部偏移规划 */
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        axis = &axis_array[n];
        // coord,teleop updates done in get_pos_cmds()
        /* 步骤 1：为外部偏移规划器设置速度与加速度限值
         * 这些限值由外部通过专用函数设置，不同于主轴的 vel_limit / acc_limit */
        axis->ext_offset_tp.max_vel = axis->ext_offset_vel_limit;
        axis->ext_offset_tp.max_acc = axis->ext_offset_acc_limit;

        axis_data = &axis_hal_data->axis[n];

        /* 步骤 2：读取外部偏移手轮的脉冲计数 */
        new_eoffset_counts       = *(axis_data->eoffset_counts);
        /* 步骤 3：计算脉冲增量 */
        delta                    = new_eoffset_counts - axis->old_eoffset_counts;
        /* 步骤 3（续）：更新旧值 */
        axis->old_eoffset_counts = new_eoffset_counts;

        /* 步骤 4：输出当前偏移量到 HAL 引脚（供外部组件读取） */
        *(axis_data->external_offset)  = axis->ext_offset_tp.curr_pos;
        /* 步骤 4（续）：使能外部偏移规划器（但后续可能根据条件再次禁用） */
        axis->ext_offset_tp.enable = 1;
        /* 步骤 5：首次调用时仅重置偏移量并跳过规划逻辑 */
        if ( first_pass ) {
            *(axis_data->external_offset) = 0;
            continue;
        }

        // Use stopping criterion of simple_tp.c:
        /* 步骤 6：计算静止判定阈值
         * TINY_DP 根据加速度和伺服周期计算一个"可忽略"的位移量
         * 当实际偏移量小于此阈值时，可以认为轴已到达目标并停止 */
        ext_offset_epsilon = TINY_DP(axis->ext_offset_tp.max_acc, servo_period);
        /* 步骤 7：如果偏移量大于阈值，标记为活跃 */
        if (fabs(*(axis_data->external_offset)) > ext_offset_epsilon) {
            eoffset_active = 1;
        }
        /* 步骤 8：处理使能信号——禁用外部偏移功能 */
        if ( !*(axis_data->eoffset_enable) ) {
            /* 禁用规划器，停止偏移运动 */
            axis->ext_offset_tp.enable = 0;
            /* 记录禁用状态 */
            last_eoffset_enable[n] = 0;
            continue;
        }
        last_eoffset_enable[n] = 1;
        /* 步骤 10：处理清除信号——将偏移目标重置为零 */
        if (*(axis_data->eoffset_clear)) {
            axis->ext_offset_tp.pos_cmd             = 0;
            *(axis_data->external_offset_requested) = 0;
            continue;
        }
        /* 步骤 11：无脉冲变化检查 */
        if (delta == 0)           { continue; }
        /* 步骤 12：回零完成检查——未完成回零时禁用外部偏移 */
        if (!all_homed)           { continue; }
        /* 步骤 13：运动使能检查 */
        if (!motion_enable_flag)  { continue; }

        /* 步骤 14：更新偏移目标位置（累积叠加） */
        axis->ext_offset_tp.pos_cmd   += delta *  *(axis_data->eoffset_scale);
        /* 步骤 15：输出请求的偏移量到 HAL 引脚 */
        *(axis_data->external_offset_requested) = axis->ext_offset_tp.pos_cmd;
    } // for n
    /* 步骤 16：清除首次调用标志 */
    first_pass = 0;

    /* 步骤 17：返回外部偏移活跃状态 */
    return eoffset_active;
}


/*
 * axis_check_constraints — 检查所有轴是否超出位置约束
 */
void axis_check_constraints(double pos[], int failing_axes[])
{
    int axis_num;
    double eps = 1e-308;  /* 极小阈值，用于检测限位值是否被设置为零（空槽位） */

    /* 遍历所有轴，检查每个轴的位置约束 */
    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num += 1) {
        /* 读取当前轴的正向和负向软限位 */
        double nl = axis_array[axis_num].min_pos_limit;  /* 负向限位（Negative Limit） */
        double pl = axis_array[axis_num].max_pos_limit;  /* 正向限位（Positive Limit） */
        /* 初始化检查结果为 0（正常） */
        failing_axes[axis_num] = 0;

        /* 步骤 2：空轴检测
         * 如果负向限位、正向限位和当前位置都接近于零，
         * 说明当前轴在配置中未启用，跳过检查
         * （fabs() 用于处理负零的情况） */
        if (   (fabs(pos[axis_num]) < eps)
            && (fabs(axis_array[axis_num].min_pos_limit) < eps)
            && (fabs(axis_array[axis_num].max_pos_limit) < eps) ) {
            continue;
        }

        /* 步骤 3：检查是否超出负向限位
         * 允许 1e-12 的微小超限容差，用于处理浮点精度误差 */
        if (pos[axis_num] < (nl - 0.000000000001)) { // see pull request #1047
            failing_axes[axis_num] = -1;
        }

        /* 步骤 4：检查是否超出正向限位
         * 允许 1e-12 的微小超限容差 */
        if (pos[axis_num] > (pl + 0.000000000001)) { // see pull request #1047
            failing_axes[axis_num] = 1;
        }
    }
}


/*
 * axis_update_coord_with_bound — 更新笛卡尔坐标并处理软限位越界
 */
int axis_update_coord_with_bound(double *pcmd_p[], double servo_period)
{
    int n;
    int ans = 0;  /* 越界轴计数器 */
    emcmot_axis_t *axis;
    /* 临时数组：保存更新前的值，用于越界时的恢复计算 */
    double save_pos_cmd[EMCMOT_MAX_AXIS];     /* 保存原始笛卡尔位置命令 */
    double save_offset_cmd[EMCMOT_MAX_AXIS];  /* 保存外部偏移目标命令 */

    /* 【第一阶段】遍历所有轴，保存旧值并更新外部偏移规划器 */
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        axis = &axis_array[n];
        /* 步骤 1：保存更新前的笛卡尔位置命令 */
        save_pos_cmd[n]     = *pcmd_p[n];
        /* 步骤 1（续）：保存更新前的外部偏移目标命令 */
        save_offset_cmd[n]  = axis->ext_offset_tp.pos_cmd;
        /* 步骤 2：调用轨迹规划器更新外部偏移的当前位置和速度
         * 规划器会根据 pos_cmd（目标）和 max_vel/max_acc（限制），
         * 在本伺服周期内计算出平滑的 curr_pos 和 curr_vel */
        simple_tp_update(&(axis->ext_offset_tp), servo_period);
    }
    /* 步骤 3：将所有轴的外部偏移量叠加到笛卡尔位置上 */
    axis_apply_ext_offsets_to_carte_pos(+1, pcmd_p); // add external offsets

    /* 【第二阶段】遍历所有轴，检查并处理软限位越界 */
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        axis = &axis_array[n];
        //workaround: axis letters not in [TRAJ]COORDINATES
        //            have min_pos_limit == max_pos_lim == 0
        /* 步骤 4：空轴跳过——配置中未启用的轴不参与限位检查 */
        if ( (0 == axis->max_pos_limit) && (0 == axis->min_pos_limit) ) {
            continue;
        }
        /* 步骤 5：无偏移时跳过——外部偏移为零时不存在越界风险 */
        if (axis->ext_offset_tp.curr_pos == 0) {
           continue; // don't claim violation if no offset
        }

        /* 步骤 6：正向软限位越界处理 */
        if (*pcmd_p[n] >= axis->max_pos_limit) {
            // hold carte_pos_cmd at the limit:
            /* 强制将笛卡尔位置截断到正向限位值 */
            *pcmd_p[n]  = axis->max_pos_limit;
            // stop growth of offsetting position:
            /* 调整外部偏移的当前位置，使其与限位后的笛卡尔位置保持一致
             * curr_pos = 限位值 - 原始笛卡尔位置 */
            axis->ext_offset_tp.curr_pos = axis->max_pos_limit
                                         - save_pos_cmd[n];
            /* 阻止偏移目标继续向越界方向增长 */
            if (axis->ext_offset_tp.pos_cmd > save_offset_cmd[n]) {
                axis->ext_offset_tp.pos_cmd = save_offset_cmd[n];
            }
            /* 将偏移速度归零，停止偏移运动 */
            axis->ext_offset_tp.curr_vel = 0;
            ans++;  /* 计数越界轴 */
            continue;
        }
        /* 步骤 7：负向软限位越界处理（与正向处理对称） */
        if (*pcmd_p[n] <= axis->min_pos_limit) {
            *pcmd_p[n]  = axis->min_pos_limit;
            axis->ext_offset_tp.curr_pos = axis->min_pos_limit
                                         - save_pos_cmd[n];
            if (axis->ext_offset_tp.pos_cmd < save_offset_cmd[n]) {
                axis->ext_offset_tp.pos_cmd = save_offset_cmd[n];
            }
            axis->ext_offset_tp.curr_vel = 0;
            ans++;
        }
    }
    /* 第四阶段：返回越界状态 */
    if (ans > 0) { return 1; }
    return 0;
}

/*
 * update_teleop_with_check — 更新轨迹规划器并检查软限位约束
 */
static int update_teleop_with_check(int axis_num, simple_tp_t *the_tp, double servo_period)
{
    // 'the_tp' is the planner to update
    // the tests herein apply to the sum of the offsets for both
    // planners (teleop_tp and ext_offset_tp)
    double save_curr_pos;  /* 保存更新前的规划器当前位置 */
    emcmot_axis_t *axis = &axis_array[axis_num];

    /* 步骤 1：保存更新前的当前位置（用于越界回退） */
    save_curr_pos = the_tp->curr_pos;
    /* 步骤 2：调用轨迹规划器的更新函数，计算新的 curr_pos 和 curr_vel */
    simple_tp_update(the_tp, servo_period);

    //workaround: axis letters not in [TRAJ]COORDINATES
    //            have min_pos_limit == max_pos_lim == 0
    /* 步骤 3：空轴跳过 */
    if  ( (0 == axis->max_pos_limit) && (0 == axis->min_pos_limit) ) {
        return 0;
    }
    /* 步骤 4：检查正向软限位越界（合成位置 = teleop + ext_offset） */
    if  ( (axis->ext_offset_tp.curr_pos + axis->teleop_tp.curr_pos)
          >= axis->max_pos_limit) {
        // positive error, restore save_curr_pos
        /* 越界检测：将规划器位置回退到上一周期的值 */
        the_tp->curr_pos = save_curr_pos;
        /* 停止规划器的运动（速度归零） */
        the_tp->curr_vel = 0;
        return 1;  /* 报告发生了限位越界 */
    }
    /* 步骤 5：检查负向软限位越界 */
    if  ( (axis->ext_offset_tp.curr_pos + axis->teleop_tp.curr_pos)
           <= axis->min_pos_limit) {
        // negative error, restore save_curr_pos
        the_tp->curr_pos = save_curr_pos;
        the_tp->curr_vel = 0;
        return 1;
    }
    return 0;  /* 未越界，正常返回 */
}

/*
 * axis_calc_motion — 执行所有轴的运动计算
 */
int axis_calc_motion(double servo_period)
{
    int axis_num;
    int violated_teleop_limit = 0;  /* 限位越界标志：至少有一个轴越界时为 1 */
    emcmot_axis_t *axis;

    /* 遍历所有轴，对每个轴执行运动计算 */
    for (axis_num = 0; axis_num < EMCMOT_MAX_AXIS; axis_num++) {
        axis = &axis_array[axis_num];
        /* 步骤 1：【防御性编程】确保遥操作规划器的速度不超过轴的限速
         * 即使点动命令设置了过高的速度，也会被强制截断到安全范围内 */
        // teleop_tp.max_vel is always positive
        if (axis->teleop_tp.max_vel > axis->vel_limit) {
            axis->teleop_tp.max_vel = axis->vel_limit;
        }
        /* 步骤 2：更新遥操作轨迹规划器并检查越界约束
         * 如果越界（返回 1），标记 violated_teleop_limit */
        if (update_teleop_with_check(axis_num, &(axis->teleop_tp), servo_period)) {
            violated_teleop_limit = 1;
        } else {
            /* 未越界时，更新轴的速度和位置输出变量 */
            /* 将规划器的当前速度输出到轴的速度命令变量 */
            axis->teleop_vel_cmd = axis->teleop_tp.curr_vel;
            /* 将规划器的当前位置输出到轴的位置命令变量 */
            axis->pos_cmd = axis->teleop_tp.curr_pos;
        }

        /* 步骤 3：当遥操作规划器停止运动时，自动清除键盘点动标志
         * 这允许手轮在点动结束后立即接管控制权 */
        if (!axis->teleop_tp.active) {
            axis->kb_ajog_active = 0;
        }

        /* 步骤 4：如果外部偏移规划器已使能，则更新外部偏移规划器
         * 外部偏移规划器独立于遥操作规划器运行，
         * 它产生的偏移量在更高层级被叠加到笛卡尔位置上 */
        if (axis->ext_offset_tp.enable) {
            /* 更新外部偏移规划器并检查越界约束 */
            if (update_teleop_with_check(axis_num, &(axis->ext_offset_tp), servo_period)) {
                violated_teleop_limit = 1;
            }
        }
    }
    /* 第五阶段：返回是否有轴发生了限位越界 */
    return violated_teleop_limit;
}
