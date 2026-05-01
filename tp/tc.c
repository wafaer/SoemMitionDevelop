#include "tc.h"
#include <stdbool.h>
#include "rtapi/rtapi.h"
#include "blendmath.h"
#include "spherical_arc.h"
#include "rtapi/rtapi_math.h"

/*
 * tcGetMaxTargetVel — 获取段的最大可达速度
 */
double tcGetMaxTargetVel(TC_STRUCT const * const tc,
        double max_scale)
{
    double v_max_target;

    switch (tc->synchronized) {
        case TC_SYNC_NONE:
            /* 无主轴同步时，速度受进给倍率（feed override）控制。
             * max_scale 通常从 UI 获取（0.0 ~ 1.0+）。 */
            v_max_target = tc->reqvel * max_scale;
            break;

        case TC_SYNC_VELOCITY: //Fallthrough
            /* 主轴速度同步模式：忽略外部速度倍率，完全由主轴转速决定。
             * max_scale 被强制设为 1.0。 */
            max_scale = 1.0;
            /* Fallthrough */
        case TC_SYNC_POSITION:
            /* 主轴位置同步模式：速度由同步算法决定，不受 feed override 影响。 */
        default:
            v_max_target = tc->maxvel;
            break;
    }

    /* 取段自身 maxvel 和计算值中的较小值。
     * 这确保了即使同步模式返回了较高的速度，也不会超过段的安全限制。 */
    return fmin(v_max_target, tc->maxvel);
}

/*
 * tcGetOverallMaxAccel — 获取段的总体最大加速度
 */
double tcGetOverallMaxAccel(const TC_STRUCT *tc)
{
    /* a_scale : 加速度缩放因子（0.0 ~ 1.0）
     * 从 1.0 开始，逐步乘以折减系数。 */
    double a_scale = (1.0 - fmax(tc->kink_accel_reduce, tc->kink_accel_reduce_prev));

    /* 【抛物线混合时的加速度折半】
     * 如果前一段有抛物线混合（blend_prev = 1），
     * 或者本段的终止条件是抛物线模式，
     * 则加速度缩放因子再乘以 0.5。
     *
     * 这是因为抛物线混合由两段对称的抛物线组成，
     * 每段各承担一半的加速度，加起来等于总加速度。 */
    if (tc->blend_prev || TC_TERM_COND_PARABOLIC == tc->term_cond) {
        a_scale *= 0.5;
    }

    return tc->maxaccel * a_scale;
}

/**
 * tcGetTangentialMaxAccel — 获取最大切向加速度
 */
double tcGetTangentialMaxAccel(TC_STRUCT const * const tc)
{
    double a_scale = tcGetOverallMaxAccel(tc);

    // 对于圆弧运动，需要考虑法向加速度
    if (tc->motion_type == TC_CIRCULAR || tc->motion_type == TC_SPHERICAL) {
        //Limit acceleration for circular arcs to allow for normal acceleration
        a_scale *= tc->acc_ratio_tan;
    }
    return a_scale;
}


/*
 * tcSetKinkProperties — 设置拐角处的运动学属性
 */
int tcSetKinkProperties(TC_STRUCT *prev_tc, TC_STRUCT *tc, double kink_vel, double accel_reduction)
{
  prev_tc->kink_vel = kink_vel;
  prev_tc->kink_accel_reduce = fmax(accel_reduction, prev_tc->kink_accel_reduce);
  tc->kink_accel_reduce_prev = fmax(accel_reduction, tc->kink_accel_reduce_prev);

  return 0;
}

/*
 * tcInitKinkProperties — 初始化拐角属性为默认值
 */
int tcInitKinkProperties(TC_STRUCT *tc)
{
    tc->kink_vel = -1.0;
    tc->kink_accel_reduce = 0.0;
    tc->kink_accel_reduce_prev = 0.0;
    return 0;
}

/*
 * tcRemoveKinkProperties — 清除拐角属性
 */
int tcRemoveKinkProperties(TC_STRUCT *prev_tc, TC_STRUCT *tc)
{
    prev_tc->kink_vel = -1.0;
    prev_tc->kink_accel_reduce = 0.0;
    tc->kink_accel_reduce_prev = 0.0;
    return 0;
}


/*
 * tcCircleStartAccelUnitVector — 计算圆弧段首的加速度方向单位向量
 */
int tcCircleStartAccelUnitVector(TC_STRUCT const * const tc, PmCartesian * const out)
{
    PmCartesian startpoint;
    PmCartesian radius;
    PmCartesian tan, perp;

    /* 获取圆弧在角度 0（起点）的位置 */
    pmCirclePoint(&tc->coords.circle.xyz, 0.0, &startpoint);
    /* 计算从圆心指向起点的半径向量 */
    pmCartCartSub(&startpoint, &tc->coords.circle.xyz.center, &radius);
    /* 切向方向 = 法向量 × 半径向量（右手定则） */
    pmCartCartCross(&tc->coords.circle.xyz.normal, &radius, &tan);
    pmCartUnitEq(&tan);
    /* 计算从起点到圆心的向量（perp 向量） */
    pmCartCartSub(&tc->coords.circle.xyz.center, &startpoint, &perp);
    pmCartUnitEq(&perp);

    /* tan 分量：方向 × 切向加速度大小 */
    pmCartScalMult(&tan, tcGetOverallMaxAccel(tc), &tan);
    /* perp 分量：单位向量 × 向心加速度大小（v²/r）
     * 其中 v = reqvel，r = 半径
     * 向心加速度 = v²/r = (reqvel²) / (2r) = (reqvel²) / (2r)
     * 0.5 因子来自于圆弧角度的微分近似。 */
    pmCartScalMultEq(&perp, pmSq(0.5 * tc->reqvel)/tc->coords.circle.xyz.radius);
    /* 两分量相加得到总加速度方向 */
    pmCartCartAdd(&tan, &perp, out);
    pmCartUnitEq(out);
    return 0;
}

/*
 * tcCircleEndAccelUnitVector — 计算圆弧段尾的加速度方向单位向量
 */
int tcCircleEndAccelUnitVector(TC_STRUCT const * const tc, PmCartesian * const out)
{
    PmCartesian endpoint;
    PmCartesian radius;

    /* 获取圆弧在角度 = angle（终点）的位置 */
    pmCirclePoint(&tc->coords.circle.xyz, tc->coords.circle.xyz.angle, &endpoint);
    pmCartCartSub(&endpoint, &tc->coords.circle.xyz.center, &radius);
    pmCartCartCross(&tc->coords.circle.xyz.normal, &radius, out);
    pmCartUnitEq(out);
    return 0;
}

/**
 * tcGetStartAccelUnitVector — 获取段首的加速度方向单位向量
 */
int tcGetStartAccelUnitVector(TC_STRUCT const * const tc, PmCartesian * const out) {

    switch (tc->motion_type) {
        case TC_LINEAR:
        case TC_RIGIDTAP:
            *out=tc->coords.line.xyz.uVec;
            break;
        case TC_CIRCULAR:
            tcCircleStartAccelUnitVector(tc,out);
            break;
        case TC_SPHERICAL:
            return -1;
        default:
            return -1;
    }
    return 0;
}

/**
 * tcGetEndAccelUnitVector — 获取段尾的加速度方向单位向量
 */
int tcGetEndAccelUnitVector(TC_STRUCT const * const tc, PmCartesian * const out) {

    switch (tc->motion_type) {
        case TC_LINEAR:
            *out=tc->coords.line.xyz.uVec;
            break;
        case TC_RIGIDTAP:
            /* 刚性攻丝段尾需要反向，所以加速度方向与段首相反 */
            pmCartScalMult(&tc->coords.line.xyz.uVec, -1.0, out);
            break;
        case TC_CIRCULAR:
            tcCircleEndAccelUnitVector(tc,out);
            break;
       case TC_SPHERICAL:
            return -1;
       default:
            return -1;
    }
    return 0;
}

/*
 * tcGetIntersectionPoint — 获取两相邻段的交点
 */
int tcGetIntersectionPoint(TC_STRUCT const * const prev_tc,
        TC_STRUCT const * const tc, PmCartesian * const point)
{
    if (tc->motion_type == TC_LINEAR) {
        *point = tc->coords.line.xyz.start;
    } else if (prev_tc->motion_type == TC_LINEAR) {
        *point = prev_tc->coords.line.xyz.end;
    } else if (tc->motion_type == TC_CIRCULAR){
        pmCirclePoint(&tc->coords.circle.xyz, 0.0, point);
    } else {
        return TP_ERR_FAIL;
    }
    return TP_ERR_OK;
}


/**
 * tcCanConsume — 判断一个段是否可以"被消费"（从队列中移除）
 */
int tcCanConsume(TC_STRUCT const * const tc)
{
    if (!tc) {
        return false;
    }

    if (tc->blend_prev || tc->atspeed) {
        return false;
    }

    return true;

}

/**
 * pmCircleTangentVector — 计算螺旋线在给定角度的切向量
 */
int pmCircleTangentVector(PmCircle const * const circle,
        double angle_in, PmCartesian * const out)
{

    PmCartesian startpoint;
    PmCartesian radius;
    PmCartesian uTan, dHelix, dRadial;

    pmCirclePoint(circle, angle_in, &startpoint);
    pmCartCartSub(&startpoint, &circle->center, &radius);

    pmCartCartCross(&circle->normal, &radius, &uTan);

    /* dz/dtheta = rHelix / angle，每单位角度的 Z 向增量 */
    double dz = 1.0 / circle->angle;
    pmCartScalMult(&circle->rHelix, dz, &dHelix);

    pmCartCartAddEq(&uTan, &dHelix);

    /* dr/dtheta = spiral / angle，每单位角度的半径变化率 */
    double dr = circle->spiral / circle->angle;
    pmCartUnit(&radius, &dRadial);
    pmCartScalMultEq(&dRadial, dr);
    pmCartCartAddEq(&uTan, &dRadial);

    pmCartUnit(&uTan, out);
    return 0;
}


/**
 * tcGetStartTangentUnitVector — 获取段首的切线方向单位向量
 */
int tcGetStartTangentUnitVector(TC_STRUCT const * const tc, PmCartesian * const out) {

    switch (tc->motion_type) {
        case TC_LINEAR:
            *out=tc->coords.line.xyz.uVec;
            break;
        case TC_RIGIDTAP:
            *out=tc->coords.rigidtap.xyz.uVec;
            break;
        case TC_CIRCULAR:
            pmCircleTangentVector(&tc->coords.circle.xyz, 0.0, out);
            break;
        default:
            (RTAPI_MSG_ERR, "Invalid motion type %d!\n",tc->motion_type);
            return -1;
    }
    return 0;
}

/**
 * tcGetEndTangentUnitVector — 获取段尾的切线方向单位向量
 */
int tcGetEndTangentUnitVector(TC_STRUCT const * const tc, PmCartesian * const out) {

    switch (tc->motion_type) {
        case TC_LINEAR:
            *out=tc->coords.line.xyz.uVec;
            break;
        case TC_RIGIDTAP:
            pmCartScalMult(&tc->coords.rigidtap.xyz.uVec, -1.0, out);
            break;
        case TC_CIRCULAR:
            pmCircleTangentVector(&tc->coords.circle.xyz,
                    tc->coords.circle.xyz.angle, out);
            break;
        default:
            rtapi_print_msg(RTAPI_MSG_ERR, "Invalid motion type %d!\n",tc->motion_type);
            return -1;
    }
    return 0;
}



/**
 * tcGetDistanceToGo — 计算轨迹段的剩余距离
 */
double tcGetDistanceToGo(TC_STRUCT const * const tc, int direction)
{
    double distance = tcGetTarget(tc, direction) - tc->progress;
    if (direction == TC_DIR_REVERSE) {
        distance *=-1.0;
    }
    return distance;
}

/*
 * tcGetTarget — 获取轨迹段的目标值
 */
double tcGetTarget(TC_STRUCT const * const tc, int direction)
{
    return (direction == TC_DIR_REVERSE) ? 0.0 : tc->target;
}

/*
 * tcGetPos — 获取 TC 沿路径的当前位置
 */
int tcGetPos(TC_STRUCT const * const tc, EmcPose * const out) {
    tcGetPosReal(tc, TC_GET_PROGRESS, out);
    return 0;
}

/*
 * tcGetStartpoint — 获取 TC 的起点
 */
int tcGetStartpoint(TC_STRUCT const * const tc, EmcPose * const out) {
    tcGetPosReal(tc, TC_GET_STARTPOINT, out);
    return 0;
}

/*
 * tcGetEndpoint — 获取 TC 的终点
 */
int tcGetEndpoint(TC_STRUCT const * const tc, EmcPose * const out) {
    tcGetPosReal(tc, TC_GET_ENDPOINT, out);
    return 0;
}

/*
 * tcGetPosReal — 核心位置计算函数
 */
int tcGetPosReal(TC_STRUCT const * const tc, int of_point, EmcPose * const pos)
{
    PmCartesian xyz;
    PmCartesian abc;
    PmCartesian uvw;
    double progress=0.0;

    /* 根据 of_point 确定使用哪个进度值 */
    switch (of_point) {
        case TC_GET_PROGRESS:        // 当前进度
            progress = tc->progress;
            break;
        case TC_GET_ENDPOINT:        // 终点
            progress = tc->target;
            break;
        case TC_GET_STARTPOINT:      // 起点
            progress = 0.0;
            break;
    }

    double angle = 0.0;
    int res_fit = TP_ERR_OK;

    /* 根据运动类型计算位置 */
    switch (tc->motion_type)
    {
        case TC_LINEAR:  //直线轨迹
            /* 直线运动：每个坐标组的进度 = 归一化进度 × 该组总长度
             * pmCartLinePoint(line, distance, &point)
             * 在直线的 tmag 方向上，距离起点 distance 处的坐标。 */
            pmCartLinePoint(&tc->coords.line.xyz,
                    progress * tc->coords.line.xyz.tmag / tc->target,
                    &xyz);
            pmCartLinePoint(&tc->coords.line.uvw,
                    progress * tc->coords.line.uvw.tmag / tc->target,
                    &uvw);
            pmCartLinePoint(&tc->coords.line.abc,
                    progress * tc->coords.line.abc.tmag / tc->target,
                    &abc);
            break;
        case TC_CIRCULAR:   //圆弧轨迹
            /* 圆弧运动：需要将弧长进度转换为角度进度
             * 因为圆弧可能是螺旋线，弧长与角度不是线性关系。
             * pmCircleAngleFromProgress() 使用二次拟合（SpiralArcLengthFit）转换。 */
            res_fit = pmCircleAngleFromProgress(&tc->coords.circle.xyz,
                    &tc->coords.circle.fit,
                    progress, &angle);
            /* 然后在角度 angle 处计算圆弧上的点 */
            pmCirclePoint(&tc->coords.circle.xyz,
                    angle,
                    &xyz);
            /* ABC 和 UVW 仍然是直线插值（同直线段） */
            pmCartLinePoint(&tc->coords.circle.abc,
                    progress * tc->coords.circle.abc.tmag / tc->target,
                    &abc);
            pmCartLinePoint(&tc->coords.circle.uvw,
                    progress * tc->coords.circle.uvw.tmag / tc->target,
                    &uvw);
            break;
        case TC_SPHERICAL:   //球面弧
            /* 球面弧：直接使用弧长进度计算点（球面弧的插值在 spherical_arc.c 中） */
            arcPoint(&tc->coords.arc.xyz,
                    progress,
                    &xyz);
            abc = tc->coords.arc.abc;
            uvw = tc->coords.arc.uvw;
            break;
    }

    /* 只有拟合成功时才更新位置 */
    if (res_fit == TP_ERR_OK) {
        // Don't touch pos unless we know the value is good
        pmCartesianToEmcPose(&xyz, &abc, &uvw, pos);
    }
    return res_fit;
}


/**
 * tcSetTermCond — 设置轨迹段的终止条件
 */
int tcSetTermCond(TC_STRUCT *prev_tc, TC_STRUCT *tc, int term_cond) {
    switch (term_cond) {
    case TC_TERM_COND_STOP:
    case TC_TERM_COND_EXACT:
    case TC_TERM_COND_TANGENT:
        if (tc) {tc->blend_prev = 0;}
        break;
    case TC_TERM_COND_PARABOLIC:
        if (tc) {tc->blend_prev = 1;}
        break;
    default:
        break;

    }
    if (prev_tc) {
        prev_tc->term_cond = term_cond;
    }
    return 0;
}


/**
 * tcConnectBlendArc — 将混合圆弧连接到两段直线
 */
int tcConnectBlendArc(TC_STRUCT * const prev_tc, TC_STRUCT * const tc,
        PmCartesian const * const circ_start,
        PmCartesian const * const circ_end) {

    if (prev_tc) {
        /* 重新初始化前一段：从原起点到圆弧起点 */
        pmCartLineInit(&prev_tc->coords.line.xyz,
                &prev_tc->coords.line.xyz.start, circ_start);
        /* 更新前一段的目标长度为缩短后的直线长度 */
        prev_tc->target = prev_tc->coords.line.xyz.tmag;
        /* 设置终止条件为切向混合 */
        tcSetTermCond(prev_tc, tc, TC_TERM_COND_TANGENT);
    } else {
    }

    /* 重新初始化后一段：从圆弧终点到原终点 */
    pmCartLineInit(&tc->coords.line.xyz, circ_end, &tc->coords.line.xyz.end);

    /* 更新后一段的目标长度 */
    tc->target = tc->coords.line.xyz.tmag;

    /* 同样设置终止条件为切向混合 */
    tcSetTermCond(prev_tc, tc, TC_TERM_COND_TANGENT);

    return 0;
}


/**
 * tcIsBlending — 判断当前段是否正在混合
 */
int tcIsBlending(TC_STRUCT * const tc) {
    int is_blending_next = (tc->term_cond == TC_TERM_COND_PARABOLIC ) &&
        tc->on_final_decel && (tc->currentvel < tc->blend_vel) &&
        tc->motion_type != TC_RIGIDTAP;

    /* 锁定：混合一旦开始就不能停止 */
    tc->blending_next |= is_blending_next;

    return tc->blending_next;
}

/*
 * tcFindBlendTolerance — 计算混合容差参数
 */
int tcFindBlendTolerance(TC_STRUCT const * const prev_tc,
        TC_STRUCT const * const tc, double * const T_blend, double * const nominal_tolerance)
{
    const double tolerance_ratio = 0.25;
    double T1 = prev_tc->tolerance;
    double T2 = tc->tolerance;
    /* 零容差 → 使用段长度的 25% 作为默认值 */
    if (T1 == 0) {
        T1 = prev_tc->nominal_length * tolerance_ratio;
    }
    if (T2 == 0) {
        T2 = tc->nominal_length * tolerance_ratio;
    }
    /* 标称容差 = 两个容差中的较小值 */
    *nominal_tolerance = fmin(T1,T2);
    /* 混合容差 = min(标称容差, 前段*0.25, 后段*0.25) */
    double blend_tolerance = fmin(fmin(*nominal_tolerance,
                prev_tc->nominal_length * tolerance_ratio),
            tc->nominal_length * tolerance_ratio);
    *T_blend = blend_tolerance;
    return 0;
}


/**
 * tcFlagEarlyStop — 检测需要提前停止的条件
 */
int tcFlagEarlyStop(TC_STRUCT * const tc,
        TC_STRUCT * const nexttc)
{

    if (!tc || !nexttc) {
        return TP_ERR_NO_ACTION;
    }

    if(tc->synchronized != TC_SYNC_POSITION && nexttc->synchronized == TC_SYNC_POSITION) {
        tcSetTermCond(tc, nexttc, TC_TERM_COND_STOP);
    }

    if(nexttc->atspeed) {
        tcSetTermCond(tc, nexttc, TC_TERM_COND_STOP);
    }

    return TP_ERR_OK;
}

/*
 * pmLine9Target — 计算直线9D运动的目标值
 */
double pmLine9Target(PmLine9 * const line9)
{
    if (!line9->xyz.tmag_zero) {
        return line9->xyz.tmag;
    } else if (!line9->uvw.tmag_zero) {
        return line9->uvw.tmag;
    } else if (!line9->abc.tmag_zero) {
        return line9->abc.tmag;
    } else {
        return 0.0;
    }
}


/**
 * tcInit — 初始化轨迹段
 *   - motion_type        : 运动类型（直线/圆弧等）
 *   - canon_motion_type  : 解释器级别的运动类型
 *   - atspeed           : 主轴到位标志
 *   - enables           : 使能位（进给倍率等）
 *   - cycle_time        : 周期时间
 *   - id                : 设为 -1（在加入队列时才分配）
 *   - indexer_jnum      : 设为 -1（无索引旋转）
 *   - active_depth      : 设为 1（活跃深度初始为1）
 *   - acc_ratio_tan     : 设为 BLEND_ACC_RATIO_TANGENTIAL（0.5）
 */
int tcInit(TC_STRUCT * const tc,
        int motion_type,
        int canon_motion_type,
        double cycle_time,
        unsigned char enables,
        char atspeed)
{

    tc->motion_type = motion_type;
    tc->canon_motion_type = canon_motion_type;
    tc->atspeed = atspeed;

    tc->enables = enables;
    tc->cycle_time = cycle_time;

    /* ID 在加入队列时才分配（可能在此之前由于混合圆弧而改变） */
    tc->id = -1; //ID to be set when added to queue (may change before due to blend arcs)

    tc->indexer_jnum = -1;

    tc->active_depth = 1;

    tc->acc_ratio_tan = BLEND_ACC_RATIO_TANGENTIAL;

    return TP_ERR_OK;
}


/**
 * tcSetupMotion — 设置轨迹段的运动学参数
 */
int tcSetupMotion(TC_STRUCT * const tc,
        double vel,
        double ini_maxvel,
        double acc)
{
    tc->maxaccel = acc;

    tc->maxvel = ini_maxvel;

    tc->reqvel = vel;
    tc->target_vel = 0;
    tcInitKinkProperties(tc);

    return TP_ERR_OK;
}

/*
 * tcSetupState — 设置轨迹段的状态参数
 */
int tcSetupState(TC_STRUCT * const tc, TP_STRUCT const * const tp)
{
    tcSetTermCond(tc, NULL, tp->termCond);
    tc->tolerance = tp->tolerance;
    tc->synchronized = tp->synchronized;
    tc->uu_per_rev = tp->uu_per_rev;
    return TP_ERR_OK;
}

/*
 * pmLine9Init — 初始化直线9D运动
 */
int pmLine9Init(PmLine9 * const line9,
        EmcPose const * const start,
        EmcPose const * const end)
{
    PmCartesian start_xyz, end_xyz;
    PmCartesian start_uvw, end_uvw;
    PmCartesian start_abc, end_abc;

    //将平移与各角向量拆分到三个 PmCartesian 里
    emcPoseToPmCartesian(start, &start_xyz, &start_abc, &start_uvw);
    emcPoseToPmCartesian(end, &end_xyz, &end_abc, &end_uvw);

    //调用 pmCartLineInit为 line9->xyz、line9->abc、line9->uvw 初始化线段信息（起点、终点、方向向量、长度等）
    int xyz_fail = pmCartLineInit(&line9->xyz, &start_xyz, &end_xyz);
    int abc_fail = pmCartLineInit(&line9->abc, &start_abc, &end_abc);
    int uvw_fail = pmCartLineInit(&line9->uvw, &start_uvw, &end_uvw);

    if (xyz_fail || abc_fail || uvw_fail) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Failed to initialize Line9, err codes %d, %d, %d\n",
                xyz_fail,abc_fail,uvw_fail);
        return TP_ERR_FAIL;
    }
    return TP_ERR_OK;
}

/*
 * pmCircle9Init — 初始化圆弧9D运动
 */
int pmCircle9Init(PmCircle9 * const circ9,
        EmcPose const * const start,
        EmcPose const * const end,
        PmCartesian const * const center,
        PmCartesian const * const normal,
        int turn)
{
    PmCartesian start_xyz, end_xyz;
    PmCartesian start_uvw, end_uvw;
    PmCartesian start_abc, end_abc;

    emcPoseToPmCartesian(start, &start_xyz, &start_abc, &start_uvw);
    emcPoseToPmCartesian(end, &end_xyz, &end_abc, &end_uvw);

    /* 初始化 XYZ 平面圆弧（可能带螺旋） */
    int xyz_fail = pmCircleInit(&circ9->xyz, &start_xyz, &end_xyz, center, normal, turn);
    //Initialize line parts of Circle9
    int abc_fail = pmCartLineInit(&circ9->abc, &start_abc, &end_abc);
    int uvw_fail = pmCartLineInit(&circ9->uvw, &start_uvw, &end_uvw);

    /* 计算螺旋圆弧的弧长拟合系数 */
    int res_fit = findSpiralArcLengthFit(&circ9->xyz,&circ9->fit);

    if (xyz_fail || abc_fail || uvw_fail || res_fit) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Failed to initialize Circle9, err codes %d, %d, %d, %d\n",
                xyz_fail, abc_fail, uvw_fail, res_fit);
        return TP_ERR_FAIL;
    }
    return TP_ERR_OK;
}

/*
 * pmCircle9Target — 计算圆弧9D的总目标距离
 */
double pmCircle9Target(PmCircle9 const * const circ9)
{

    double h2;
    pmCartMagSq(&circ9->xyz.rHelix, &h2);
    /* 螺旋长度 = sqrt(平面弧长² + 螺旋高度²) */
    double helical_length = pmSqrt(pmSq(circ9->fit.total_planar_length) + h2);

    return helical_length;
}

/*
 * tcUpdateCircleAccRatio — 更新圆弧的加速度比率
 */
int tcUpdateCircleAccRatio(TC_STRUCT * tc)
{
    if (tc->motion_type == TC_CIRCULAR) {
        PmCircleLimits limits = pmCircleActualMaxVel(&tc->coords.circle.xyz,
                             tc->maxvel,
                             tcGetOverallMaxAccel(tc));
        tc->maxvel = limits.v_max;
        tc->acc_ratio_tan = limits.acc_ratio;
        return 0;
    }
    return 1; //nothing to do, but not an error
}

/**
 * tcFinalizeLength — 定稿轨迹段长度
 */
int tcFinalizeLength(TC_STRUCT * const tc)
{
    //Apply velocity corrections
    if (!tc) {
        return TP_ERR_FAIL;
    }

    if (tc->finalized) {
        return TP_ERR_NO_ACTION;
    }

    tcClampVelocityByLength(tc);

    tcUpdateCircleAccRatio(tc);

    tc->finalized = 1;
    return TP_ERR_OK;
}


/*
 * tcClampVelocityByLength — 根据段长度限制速度
 */
int tcClampVelocityByLength(TC_STRUCT * const tc)
{
    //Apply velocity corrections
    if (!tc) {
        return TP_ERR_FAIL;
    }

    double sample_maxvel = tc->target / tc->cycle_time;
    tc->maxvel = fmin(tc->maxvel, sample_maxvel);
    return TP_ERR_OK;
}

/**
 * tcUpdateTargetFromCircle — 从圆弧几何更新段目标
 */
int tcUpdateTargetFromCircle(TC_STRUCT * const tc)
{
    if (!tc || tc->motion_type !=TC_CIRCULAR) {
        return TP_ERR_FAIL;
    }

    double h2;
    pmCartMagSq(&tc->coords.circle.xyz.rHelix, &h2);
    /* 螺旋总弧长 = sqrt(平面弧长² + 螺旋高度²) */
    double helical_length = pmSqrt(pmSq(tc->coords.circle.fit.total_planar_length) + h2);

    tc->target = helical_length;
    return TP_ERR_OK;
}



/*
 * pmRigidTapInit — 初始化刚性攻丝运动
 */
int pmRigidTapInit(PmRigidTap * const tap,
        EmcPose const * const start,
        EmcPose const * const end,
        double reversal_scale)
{
    PmCartesian start_xyz, end_xyz;
    PmCartesian abc, uvw;

    emcPoseToPmCartesian(start, &start_xyz, &abc, &uvw);
    emcPoseGetXYZ(end, &end_xyz);

    // 初始化 XYZ 直线运动（从起点到终点） */
    pmCartLineInit(&tap->xyz, &start_xyz, &end_xyz);

    // ABC 和 UVW 在刚性攻丝过程中保持固定 */
    tap->abc = abc;
    tap->uvw = uvw;

    // reversal_target 是攻丝底部的目标距离 */
    tap->reversal_target = tap->xyz.tmag;
    tap->reversal_scale = reversal_scale;
    tap->state = RIGIDTAP_START;
    return TP_ERR_OK;

}

/*
 * pmRigidTapTarget — 计算刚性攻丝的总目标距离
 */
double pmRigidTapTarget(PmRigidTap * const tap, double uu_per_rev)
{
    /* 假设主轴需要 10 转才能完全停止 */
    double overrun = 10. * uu_per_rev;
    double target = tap->xyz.tmag + overrun;
    return target;
}

/*
 * tcPureRotaryCheck — 检查是否仅有旋转运动
 */
int tcPureRotaryCheck(TC_STRUCT const * const tc)
{
    return (tc->motion_type == TC_LINEAR) &&
        (tc->coords.line.xyz.tmag_zero) &&
        (tc->coords.line.uvw.tmag_zero);
}


/**
 * tcSetCircleXYZ — 用新的圆弧几何替换 TC 中的 XYZ 圆弧
 */
int tcSetCircleXYZ(TC_STRUCT * const tc, PmCircle const * const circ)
{

    //Update targets with new arc length
    if (!circ || tc->motion_type != TC_CIRCULAR) {
        return TP_ERR_FAIL;
    }
    if (!tc->coords.circle.abc.tmag_zero || !tc->coords.circle.uvw.tmag_zero) {
        rtapi_print_msg(RTAPI_MSG_ERR, "SetCircleXYZ does not supportABC or UVW motion\n");
        return TP_ERR_FAIL;
    }

    if (!circ) {
        rtapi_print_msg(RTAPI_MSG_ERR, "SetCircleXYZ missing new circle definition\n");
        return TP_ERR_FAIL;
    }

    /* 替换圆弧数据 */
    tc->coords.circle.xyz = *circ;
    /* 重新计算螺旋拟合系数 */
    findSpiralArcLengthFit(&tc->coords.circle.xyz, &tc->coords.circle.fit);

    // 重新计算总目标距离 */
    tc->target = pmCircle9Target(&tc->coords.circle);

    return TP_ERR_OK;
}

/*
 * tcClearFlags — 清除 TC 的临时状态标志
 */
int tcClearFlags(TC_STRUCT * const tc)
{
    if (!tc) {
        return TP_ERR_MISSING_INPUT;
    }

    tc->is_blending = false;

    return TP_ERR_OK;
}
