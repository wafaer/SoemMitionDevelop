#include "simple_tp.h"
#include "rtapi/rtapi_math.h"
#include "rtapi/rtapi.h"

/*
 * simple_tp_update — 更新单轴轨迹规划器
 */

void simple_tp_update(simple_tp_t *tp, int32_t period)
{
    double max_dv, tiny_dp, pos_err, vel_req;

    /* max_dv : 每个伺服周期的最大速度增量
     * 计算公式：max_dv = acc * period
     * 含义：如果在一个伺服周期内从零加速到最大速度，速度增量为 acc * period。
     * active 预先清零。如果后续发现需要运动，会重新置 1。
     * 这是"保守优先"的设计：先假设静止，再根据实际情况更新。 */
    tp->active = 0;
    max_dv = tp->acc * period;
    /* tiny_dp : 可忽略的位置阈值。
     * 当 |pos_err| < tiny_dp 时，认为轴已经"到达"目标，停止运动。
     * 计算公式：tiny_dp = acc * period² * 0.001
     * 物理含义：在 tiny_dp 这么小的位移内，以加速度 acc 减速到零，
     *          所需的初始速度刚好等于 max_dv（一个周期的速度增量）。
     *          因此小于此阈值的运动没有实际意义。*/
    tiny_dp = TINY_DP(tp->acc, period);
    /* 步骤 2：计算期望速度（vel_req）
     *
     * vel_req 是规划器希望达到的"理想速度"。
     * 它的计算遵循以下原则：
     *   1. 尽量缩短到达目标的时间（以最大加速度加速）。
     *   2. 保证不会超调（接近目标时以最大加速度减速）。
     *   3. 不超过最大速度限制（后续步骤处理）。
     */
    if (tp->enable) {
        /* 当规划器被使能时，计算驱动位置误差趋于零的速度请求，
         * 同时保证在当前位置不会超调 */

        /* 计算位置误差：目标位置 - 当前位置 */
        pos_err = tp->pos_cmd - tp->curr_pos;

        /* 处理正向运动：pos_err > tiny_dp */
        /* 当目标在当前位置的正向，且距离大于 tiny_dp 时，
         * 使用正向运动公式计算期望速度 */
        if (pos_err > tiny_dp)
        {
            /* 正向速度请求公式：
             * vel_req = -max_dv + sqrt(2 * acc * pos_err + max_dv²)
             *
             * 【数学验证】
             *   令 v = vel_req + max_dv
             *   则 v² = 2 * acc * pos_err + max_dv²
             *   即 v = sqrt(2 * acc * pos_err + max_dv²)
             *
             *   如果 curr_vel = 0（从静止开始）：
             *     curr_vel 每周期增加 max_dv，经过 n 个周期后：
             *       curr_vel_n = n * max_dv
             *       pos_n = sum(curr_vel_i) * period = n*(n+1)/2 * max_dv * period
             *             ≈ n²/2 * acc * period²
             *     当 curr_vel_n 接近 vel_req 时，有：
             *       curr_vel_n² ≈ 2 * acc * pos_n
             *     这保证了在加速过程中不会超调。*/
            vel_req = -max_dv + sqrt(2 * tp->acc * pos_err + max_dv * max_dv);
            /* 标记规划器处于活跃状态（正在运动） */
            tp->active = 1;
        }
        /* 处理负向运动：pos_err < -tiny_dp */
        /* 当目标在当前位置的负向，且距离大于 tiny_dp 时，
         * 使用负向运动公式计算期望速度 */
        else if (pos_err < -tiny_dp)
        {
            /* 负向速度请求公式：
             * vel_req = max_dv - sqrt(2 * acc * |pos_err| + max_dv²)
             *
             * 通过改变 max_dv 的符号，实现负向运动。
             * 公式确保 vel_req 为负值，且绝对值随 |pos_err| 增大而增大。*/
            vel_req =  max_dv - sqrt(2 * tp->acc * pos_err + max_dv * max_dv);
            /* mark planner as active */
            tp->active = 1;
        } else {
            /* 当 |pos_err| <= tiny_dp 时，认为已经"足够接近"目标，
             * 无需继续运动，将速度请求设为零。 */
            /* within 'tiny_dp' of desired pos, no need to move */
            vel_req = 0;
            /* 自动禁用规划器。
             * 当轴到达目标后，清除 enable 标志，使得下次调用时进入 else 分支。
             * 这是自动停止机制：无需外部干预，规划器自动停止。*/
            tp->enable = 0;
        }

    } else {
        /* 当规划器被禁用（enable == 0）时，
         * 请求零速度，使轴减速到停止。*/
        /* planner disabled, request zero velocity */
        vel_req = 0;
        /* 将目标位置命令同步到当前位置。
         * 这确保了当下次使能规划器时，目标位置与当前位置一致，
         * 避免了 disable 期间由于某种原因（外部扰动等）curr_pos 发生变化后，
         * 重新 enable 时出现位置跳变。*/
        tp->pos_cmd = tp->curr_pos;
    }

    /* 【速度限幅阶段】
     * 将计算出的期望速度 vel_req 限制在 [-vel, +vel] 范围内。
     * vel 通常等于 max_vel，但也可以是较小的值。
     * 这样可以灵活地限制运动速度，而不需要修改 max_vel。 */

    /* 步骤 3：速度限幅
     * 将 vel_req 限制在允许的速度范围内 */
    if (vel_req > tp->vel) {
        vel_req = tp->vel;
    } else if (vel_req < -tp->vel) {
        vel_req = -tp->vel;
    }

    /* 【速度斜坡更新阶段】
     * curr_vel 不能突变，必须以加速度 acc 限制的速度增量（max_dv）向 vel_req 逼近。
     * 这就是"斜坡"（ramp）——速度变化被限制在一个固定的斜率上。 */

    /* ramp velocity toward request at accel limit
     * 如果期望速度大于当前速度加上一个周期的最大增量，
     * 则以最大加速度（每周期 max_dv）加速 */
    if (vel_req > tp->curr_vel + max_dv) {
        tp->curr_vel += max_dv;
    }
    /* 如果期望速度小于当前速度减去一个周期的最大增量，
     * 则以最大减速度减速 */
    else if (vel_req < tp->curr_vel - max_dv) {
        tp->curr_vel -= max_dv;
    }
    /* 否则（当前速度已在允许范围内），直接跳到期望速度 */
    else {
        tp->curr_vel = vel_req;
    }

    /* 【运动活跃检测】
     * 最后再检查一次：如果 curr_vel 不为零，说明仍在运动。 */

    /* check for still moving
     * 如果当前速度不为零，说明轴仍在运动。
     * 注意：即使 active 之前被置为 1，这里也再次确认。
     * 原因：即使 vel_req = 0（目标速度为零），curr_vel 仍然可能不为零
     * （减速过程中），所以需要再次标记为活跃。 */
    if (tp->curr_vel != 0) {
        /* yes, mark planner active */
        tp->active = 1;
    }

    /* 【位置积分阶段】
     * 最后一步：使用欧拉积分法更新当前位置。
     * curr_pos_new = curr_pos_old + curr_vel * period
     * 这是最简单的一阶积分，适合伺服周期足够小的情况。 */

    /* 当前位置累加：curr_pos += curr_vel * period
     * 速度 × 时间 = 位移 */
    tp->curr_pos += tp->curr_vel * period;
}
