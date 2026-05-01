//
// Created by Administrator on 2025/8/16.
//

/*
 * simple_tp.h — 单轴轨迹规划器数据结构与宏定义
 * =========================================================
 *
 * 一、模块概述
 * ------------
 * simple_tp（Simple Trajectory Planner，单轴轨迹规划器）是 LinuxCNC 运动控制
 * 子系统中最基础的轨迹规划组件。
 *
 * 它为单个轴（Joint）提供一维位置轨迹规划功能：将一个目标位置（pos_cmd）
 * 作为输入，在给定的最大速度和最大加速度约束下，自动生成平滑的速度曲线，
 * 使轴从当前位置（curr_pos）移动到目标位置（pos_cmd）。
 *
 * 二、运动模式分析
 * ----------------
 * simple_tp 产生的速度轮廓既不是纯梯形（ trapezoidal），也不是纯 S 曲线，
 * 而是一种"渐进加速/减速"模式：
 *
 *   速度
 *   ^
 *   |        /\
 *   |       /  \
 *   |      /    \
 *   |     /      \
 *   |    /        \
 *   |---/----------\------------------> 位置
 *        ↑        ↑
 *      curr_pos  pos_cmd
 *
 * 【关键特点】
 *   1. 以最大加速度加速，直到达到最大速度或接近目标。
 *   2. 到达目标前以最大加速度减速。
 *   3. 如果目标距离很近，可能永远不会达到最大速度。
 *   4. 如果当前位置已在目标点的 tiny_dp 范围内，立即停止。
 *
 * 三、核心算法：三角/梯形速度规划
 * --------------------------------
 * simple_tp 使用了一种简洁而优雅的速度规划算法，避免了复杂的分段处理：
 *
 *   【正向运动时的速度请求计算】
 *     vel_req = -max_dv + sqrt(2 * acc * pos_err + max_dv²)
 *
 *   【负向运动时的速度请求计算】
 *     vel_req = +max_dv - sqrt(2 * acc * |pos_err| + max_dv²)
 *
 *   其中：
 *     - pos_err = pos_cmd - curr_pos（位置误差）
 *     - max_dv  = acc * period（每个伺服周期的最大速度增量）
 *
 *   【公式推导】
 *     假设从零速度开始，以加速度 acc 做匀加速运动，经过 n 个周期后：
 *       速度 v_n = n * max_dv
 *       位移 s_n = (n²/2) * max_dv * period + n * max_dv * max_dv / 2
 *       整理后：s_n = (n² * acc * period²)/2 + (n * acc * period)² / 2
 *     反解 n：n = (sqrt(2 * acc * s_n + max_dv²) - max_dv) / max_dv
 *     最终速度：v_n = sqrt(2 * acc * s_n + max_dv²) - max_dv
 *     即：vel_req = sqrt(2 * acc * pos_err + max_dv²) - max_dv
 *
 * 四、TINY_DP 宏的作用
 * --------------------
 * TINY_DP 用来计算一个"可忽略的位置阈值"：
 *   TINY_DP = max_acc * period² * 0.001
 *
 * 【为什么需要这个阈值？】
 *   1. 防止浮点精度问题（接近目标时 sqrt(2*acc*pos_err) 可能不稳定）。
 *   2. 当位置误差小于此阈值时，直接视为"已到达"，避免无谓的微小运动。
 *   3. 物理含义：在这个位移内，以最大加速度减速到零所需的速度增量。
 *
 *   例如：acc = 1000 mm/s², period = 0.001 s
 *   则 tiny_dp = 1000 * 0.001² * 0.001 = 0.000001 mm
 *   即 1 微米。
 */

#ifndef SIMPLE_TP_H
#define SIMPLE_TP_H
#include <stdint.h>

/*
 * TINY_DP — 计算"可忽略位置阈值"宏
 * ----------------------------------
 * 【宏原型】
 *   #define TINY_DP(max_acc, period) (max_acc * period * period * 0.001)
 *
 * 【参数】
 *   - max_acc : 最大加速度（用户单位/秒²）
 *   - period  : 伺服周期（秒）
 *
 * 【返回值】
 *   一个极小的位置值（用户单位）。
 *
 * 【数学含义】
 *   在 tiny_dp 这么小的位移范围内，以加速度 acc 进行匀减速到零时，
 *   所需的初始速度为 max_dv（一个伺服周期内的最大速度增量）。
 *   换言之：当位置误差 < tiny_dp 时，即使继续运动，下一个周期的速度增量
 *   也只有 max_dv 量级，继续运动没有意义。
 *
 * 【用途】
 *   1. 作为"到达判定"的容差。
 *   2. 避免在目标点附近出现微小的振荡或过冲。
 *   3. 消除 sqrt() 函数中 tiny input 导致的数值不稳定。
 *
 * 【数值示例】
 *   设 acc = 1000 mm/s², period = 0.001 s (1ms)
 *   则 TINY_DP = 1000 * 0.001 * 0.001 * 0.001 = 1e-6 mm = 1 微米
 *   这意味着当目标位置距离当前值小于 1 微米时，规划器认为已经"到达"。
 */
#define TINY_DP(max_acc,period) (max_acc*period*period*0.001)

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * simple_tp_t — 单轴轨迹规划器数据结构
     * ------------------------------------
     *
     * 【结构体作用】
     *   保存单个轴的一维轨迹规划器的全部状态。
     *   在每个伺服周期内，调用 simple_tp_update() 来更新规划器状态。
     *
     * 【设计理念】
     *   - pos_cmd / max_vel / max_acc : 由外部设置的目标参数（只读）。
     *   - vel / acc                   : 由外部设置的次级参数（通常是 pos_cmd 的倍数）。
     *   - curr_pos / curr_vel         : 由规划器更新的输出（只写）。
     *   - enable                       : 控制信号（0=停止，1=启动）。
     *   - active                      : 状态标志（0=静止，1=运动中）。
     *   - dir                          : 方向标志（备用字段）。
     */
    typedef struct simple_tp_t
    {
        double pos_cmd;		/* position command（目标位置）
                            【含义】轴应该到达的目标位置（用户单位）。
                            【写入者】通常由 command.c 中的点动命令或轨迹规划器设置。
                            【读取者】simple_tp_update() 读取此值来计算位置误差。*/

        double max_vel;		/* velocity limit（最大速度限制）
                            【含义】本运动允许的最大速度（用户单位/秒）。
                            【写入者】通常由 command.c 设置。
                            【用途】在速度请求计算的最后阶段，对 vel_req 进行限幅。*/

        double max_acc;		/* acceleration limit（最大加速度限制）
                            【含义】本运动允许的最大加速度（用户单位/秒²）。
                            【写入者】通常由 command.c 设置。
                            【用途】用于计算 max_dv（每个周期的速度增量）。*/

        double vel;		/* velocity（运动速度参数）
                            【含义】本运动的允许速度（通常等于 max_vel 或其倍数）。
                            【注意】这个字段的命名容易与 curr_vel 混淆。
                            从代码逻辑看，vel 是"允许的速度上限"，curr_vel 是"当前实际速度"。*/

        double acc;		/* acceleration（运动加速度参数）
                            【含义】本运动的允许加速度（通常等于 max_acc）。
                            【注意】acc 与 max_acc 是不同的字段，acc 可能被设为较小的值。*/

        int enable;		/* if zero, motion stops ASAP（使能标志）
                            【含义】规划器的使能控制。
                            = 0 : 规划器被禁用，请求零速度，并在当前位置停止。
                            = 1 : 规划器被使能，根据 pos_cmd 计算运动。
                            【写入者】command.c 中的点动命令和 axis.c 中的 abort 函数。
                            【注意】当轴到达目标位置时（|pos_err| <= tiny_dp），
                                   enable 会被自动清零（见第 39 行）。*/

        double curr_pos;	/* current position（当前位置）
                            【含义】规划器当前计算出的位置（用户单位）。
                            【写入者】simple_tp_update() 每周期更新此值。
                            【读取者】command.c、control.c 读取此值作为轴的位置命令。*/

        double curr_vel;	/* current velocity（当前速度）
                            【含义】规划器当前的速度值（用户单位/秒）。
                            【写入者】simple_tp_update() 在速度斜坡阶段更新此值。
                            【读取者】control.c 读取此值作为轴的速度命令。*/

        int active;		/* non-zero if motion in progress（运动活跃标志）
                            【含义】标记规划器是否正在进行运动。
                            = 0 : 规划器处于静止状态（到达目标或被禁用）。
                            = 1 : 规划器正在进行运动（尚未到达目标）。
                            【写入者】simple_tp_update()。
                            【读取者】command.c、control.c 用于判断是否可以切换模式。*/

        int dir;          /* direction（运动方向）
                            【含义】预留的运动方向标志（备用字段）。*/
    } simple_tp_t;

    /*
     * simple_tp_update — 更新单轴轨迹规划器状态
     * ----------------------------------------
     * 函数声明，定义于 simple_tp.c 中。
     */
    extern void simple_tp_update(simple_tp_t *tp, int32_t period);


#ifdef __cplusplus
}
#endif

#endif //SIMPLE_TP_H
