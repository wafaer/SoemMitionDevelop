#include "tp.h"
#include "libnml/emcpos.h"
#include <math.h>
#include <stdbool.h>
#include "blendmath.h"
#include "spherical_arc.h"
#include "tc.h"
#include "rtapi/rtapi.h"
#include "rtapi/rtapi_mutex.h"

/*
 * queueTcSpace — 轨迹段队列的预分配存储空间
 */
TC_STRUCT queueTcSpace[DEFAULT_TC_QUEUE_SIZE + 10];

extern emcmot_status_t *emcmotStatus;
extern emcmot_config_t *emcmotConfig;
extern emcmot_internal_t *emcmotInternal;

static double(*_axis_get_vel_limit)(int);
static double(*_axis_get_acc_limit)(int);
static void(  *_SetRotaryUnlock)(int,int);
static int (  *_GetRotaryIsUnlocked)(int);

/*
 * tpMotFunctions — 注册运动学回调函数
 */
void tpMotFunctions(void(  *pSetRotaryUnlock)(int,int) ,int (  *pGetRotaryIsUnlocked)(int), double(*paxis_get_vel_limit)(int),double(*paxis_get_acc_limit)(int))
{
    _SetRotaryUnlock     = pSetRotaryUnlock;
    _GetRotaryIsUnlocked = pGetRotaryIsUnlocked;
    _axis_get_vel_limit  = paxis_get_vel_limit;
    _axis_get_acc_limit  = paxis_get_acc_limit;
}

/*
 * tpMotData — 注册共享内存指针
 */
void tpMotData(emcmot_status_t *pstatus,emcmot_config_t *pconfig)
{
    emcmotStatus = pstatus;
    emcmotConfig = pconfig;
}

/*
 * tpCreate — 创建轨迹规划器
 */
int tpCreate(TP_STRUCT * const tp, int _queueSize,int id)
{
    (void)id;
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    if (_queueSize <= 0) {
        tp->queueSize = TP_DEFAULT_QUEUE_SIZE;
    } else {
        tp->queueSize = _queueSize;
    }
    /* 队列存储空间是预分配的静态数组，非动态分配 */
    TC_STRUCT * const tcSpace = queueTcSpace;

    if (-1 == tcqCreate(&tp->queue, tp->queueSize, tcSpace))
    {
        return TP_ERR_FAIL;
    }

    return tpInit(tp);
}

/*
 * tpClear — 清空轨迹规划器
 */
int tpClear(TP_STRUCT * const tp) {
    tcqInit(&tp->queue);
    tp->queueSize = 0;
    tp->goalPos = tp->currentPos;
    tp->nextId = 0;
    tp->execId = 0;
    tp->motionType = 0;
    tp->done = 1;
    tp->depth = tp->activeDepth = 0;
    tp->aborting = 0;
    tp->pausing = 0;
    tp->reverse_run = 0;
    tp->synchronized = 0;
    tp->uu_per_rev = 0.0;
    emcmotStatus->current_vel = 0.0;
    emcmotStatus->requested_vel = 0.0;
    emcmotStatus->distance_to_go = 0.0;

    return 0;
}

/*
 * tcRotaryMotionCheck — 检查段是否包含旋转轴运动
 */
static int tcRotaryMotionCheck(TC_STRUCT const * const tc) {
    switch (tc->motion_type) {
        case TC_RIGIDTAP:
            return false;
        case TC_LINEAR:
            if (tc->coords.line.abc.tmag_zero && tc->coords.line.uvw.tmag_zero) {
                return false;
            } else {
                return true;
            }
        case TC_CIRCULAR:
            if (tc->coords.circle.abc.tmag_zero && tc->coords.circle.uvw.tmag_zero) {
                return false;
            } else {
                return true;
            }
        case TC_SPHERICAL:
            return true;
        default:

        return false;
    }
}

/*
 * getMaxFeedScale — 获取最大进给缩放因子
 */
static inline double getMaxFeedScale(TC_STRUCT const * tc)
{
    if (tc && tc->synchronized == TC_SYNC_POSITION ) {
        return 1.0;
    } else {
        return emcmotConfig->maxAxisScale;
    }
}

/*
 * tpGetMachineAccelBounds — 获取机器各轴加速度限制
 */
static int tpGetMachineAccelBounds(PmCartesian  * const acc_bound) {
    if (!acc_bound)
    {
        return TP_ERR_FAIL;
    }

    acc_bound->x = _axis_get_acc_limit(0); //0==>x
    acc_bound->y = _axis_get_acc_limit(1); //1==>y
    acc_bound->z = _axis_get_acc_limit(2); //2==>z
    return TP_ERR_OK;
}

/*
 * tpGetTangentKinkRatio — 获取切向拐角比率
 */
static double tpGetTangentKinkRatio(void) {
    const double max_ratio = 0.7071;
    const double min_ratio = 0.001;

    return fmax(fmin(emcmotConfig->arcBlendTangentKinkRatio,max_ratio),min_ratio);
}

/*
 * tpGetFeedScale — 获取当前有效的进给缩放因子
 */
static double tpGetFeedScale(TP_STRUCT const * const tp, TC_STRUCT const * const tc) {
    if (!tc) {
        return 0.0;
    }
    bool pausing = tp->pausing && (tc->synchronized == TC_SYNC_NONE || tc->synchronized == TC_SYNC_VELOCITY);
    bool aborting = tp->aborting;
    if (pausing)  {
        return 0.0;
    } else if (aborting) {
        return 0.0;
    } else if (tc->synchronized == TC_SYNC_POSITION ) {
        return 1.0;
    } else if (tc->is_blending) {
        return fmin(emcmotStatus->net_feed_scale, 1.0);
    } else {
        return emcmotStatus->net_feed_scale;
    }
}

/*
 * tpGetMaxTargetVel — 获取目标最大速度
 */
static inline double tpGetMaxTargetVel(TP_STRUCT const * const tp, TC_STRUCT const * const tc)
{
    double max_scale = emcmotConfig->maxAxisScale;
    if (tc->is_blending) {
        max_scale = fmin(max_scale, 1.0);
    }
    double v_max_target = tcGetMaxTargetVel(tc, max_scale);

    if (!tcPureRotaryCheck(tc) && (tc->synchronized != TC_SYNC_POSITION)){
        v_max_target = fmin(v_max_target, tp->vLimit);
    }
    return v_max_target;
}

/*
 * tpGetRealTargetVel — 获取实际目标速度
 */
static inline double tpGetRealTargetVel(TP_STRUCT const * const tp,
        TC_STRUCT const * const tc) {

    if (!tc) {
        return 0.0;
    }
    double v_target = tc->synchronized ? tc->target_vel : tc->reqvel;

    return fmin(v_target * tpGetFeedScale(tp,tc), tpGetMaxTargetVel(tp, tc));
}

/*
 * tpGetMachineVelBounds — 获取机器各轴速度限制
 */
static int tpGetMachineVelBounds(PmCartesian  * const vel_bound) {
    if (!vel_bound) {
        return TP_ERR_FAIL;
    }

    vel_bound->x = _axis_get_vel_limit(0); //0==>x
    vel_bound->y = _axis_get_vel_limit(1); //1==>y
    vel_bound->z = _axis_get_vel_limit(2); //2==>z
    return TP_ERR_OK;
}

/*
 * tpGetMachineActiveLimit — 获取激活轴中的最小限制值
 */
static int tpGetMachineActiveLimit(double * const act_limit, PmCartesian const * const bounds) {
    if (!act_limit) {
        return TP_ERR_FAIL;
    }
    *act_limit = fmax(fmax(bounds->x,bounds->y),bounds->z);

    if (bounds->x > 0) {
        *act_limit = fmin(*act_limit, bounds->x);
    }
    if (bounds->y > 0) {
        *act_limit = fmin(*act_limit, bounds->y);
    }
    if (bounds->z > 0) {
        *act_limit = fmin(*act_limit, bounds->z);
    }
    return TP_ERR_OK;
}

/*
 * tpGetRealFinalVel — 获取段的真实最终速度
 */
static inline double tpGetRealFinalVel(TP_STRUCT const * const tp,
    TC_STRUCT const * const tc, TC_STRUCT const * const nexttc) {

    double v_target = 0.0;
    if (emcmotStatus->stepping || tc->term_cond != TC_TERM_COND_TANGENT || tp->reverse_run) {
        return 0.0;
    }

    double v_target_this = tpGetRealTargetVel(tp, tc);

    double v_target_next = 0.0;
    if (nexttc) {
        v_target_next = tpGetRealTargetVel(tp, nexttc);
        v_target = fmin(v_target_this, v_target_next);
    }else
    {
        v_target = v_target_this;
    }

    return fmin(tc->finalvel, v_target);
}

/*
 * tpInit — 初始化轨迹规划器
 */
int tpInit(TP_STRUCT * const tp)
{
    tp->cycleTime = 0.0;
    tp->vLimit = 0.0;
    tp->ini_maxvel = 0.0;
    tp->aLimit = 0.0;
    PmCartesian acc_bound;
    if (emcmotStatus == 0) {
       rtapi_print_msg(RTAPI_MSG_ERR, "!!!tpInit: NULL emcmotStatus, bye\n\n");
       return -1;
    }
    tpGetMachineAccelBounds(&acc_bound);
    tpGetMachineActiveLimit(&tp->aMax, &acc_bound);
    tp->wMax = 0.0;
    tp->wDotMax = 0.0;

    tp->reverse_run = TC_DIR_FORWARD;
    tp->termCond = TC_TERM_COND_TANGENT;
    tp->tolerance = 0.0;

    ZERO_EMC_POSE(tp->currentPos);

    PmCartesian vel_bound;
    tpGetMachineVelBounds(&vel_bound);
    tpGetMachineActiveLimit(&tp->vMax, &vel_bound);

    return tpClear(tp);
}

/**
 * tpSetCycleTime — 设置伺服周期时间
 */
int tpSetCycleTime(TP_STRUCT * const tp, double secs)
{
    if (0 == tp || secs <= 0.0) {
        return TP_ERR_FAIL;
    }

    tp->cycleTime = secs;

    return TP_ERR_OK;
}

/**
 * tpSetVmax — 设置最大速度和初始最大速度
 */
int tpSetVmax(TP_STRUCT * const tp, double vMax, double ini_maxvel)
{
    if (0 == tp || vMax <= 0.0 || ini_maxvel <= 0.0) {
        return TP_ERR_FAIL;
    }

    tp->vMax = vMax;
    tp->ini_maxvel = ini_maxvel;

    return TP_ERR_OK;
}

/**
 * tpSetVlimit — 设置笛卡尔最大速度限制
 */
int tpSetVlimit(TP_STRUCT * const tp, double vLimit)
{
    if (!tp) return TP_ERR_FAIL;

    if (vLimit < 0.)
        tp->vLimit = 0.;
    else
        tp->vLimit = vLimit;

    return TP_ERR_OK;
}

/** tpSetAmax — 设置最大加速度 */
int tpSetAmax(TP_STRUCT * const tp, double aMax)
{
    if (0 == tp || aMax <= 0.0) {
        return TP_ERR_FAIL;
    }

    tp->aMax = aMax;

    return TP_ERR_OK;
}

/**
 * tpSetId — 设置下一个轨迹段的运动 ID
 */
int tpSetId(TP_STRUCT * const tp, int id)
{

    if (!MOTION_ID_VALID(id)) {
        rtapi_print_msg(RTAPI_MSG_ERR, "tpSetId: invalid motion id %d\n", id);
        return TP_ERR_FAIL;
    }

    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    tp->nextId = id;

    return TP_ERR_OK;
}

/** tpGetExecId — 获取当前正在执行的段的 ID */
int tpGetExecId(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    return tp->execId;
}


/**
 * tpSetTermCond — 设置终止条件和容差
 * 为所有后续排队移动设置终止条件
 * 若条件为 TC_TERM_COND_STOP，则当前移动将在后续移动开始前停止
 * 若条件为 TC_TERM_COND_PARABOLIC，则当当前移动减速至低于计算出的过渡速度时，将启动后续移动
 */
int tpSetTermCond(TP_STRUCT * const tp, int cond, double tolerance)
{
    if (!tp) {
        return TP_ERR_FAIL;
    }

    switch (cond) {
        case TC_TERM_COND_PARABOLIC:
        case TC_TERM_COND_TANGENT:
        case TC_TERM_COND_EXACT:
        case TC_TERM_COND_STOP:
            tp->termCond = cond;
            tp->tolerance = tolerance;
            break;
        default:
            return  -1;
    }

    return TP_ERR_OK;
}

/**
 * tpSetPos — 设置规划器当前位置和目标位置
 * 仅在 TP 初始化和模式切换时使用
 */
int tpSetPos(TP_STRUCT * const tp, EmcPose const * const pos)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    int res_invalid = tpSetCurrentPos(tp, pos);
    if (res_invalid) {
        return TP_ERR_FAIL;
    }

    tp->goalPos = *pos;
    return TP_ERR_OK;
}


/**
 * tpSetCurrentPos — 设置当前位姿
 */
int tpSetCurrentPos(TP_STRUCT * const tp, EmcPose const * const pos)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    if (emcPoseValid(pos)) {
        tp->currentPos = *pos;
        return TP_ERR_OK;
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR, "Tried to set invalid pose in tpSetCurrentPos on id %d!" "pos is %.12g, %.12g, %.12g\n",
                tp->execId,
                pos->tran.x,
                pos->tran.y,
                pos->tran.z);
        return TP_ERR_INVALID;
    }
}


/*
 * tpAddCurrentPos — 将位移增量加到当前位置
 */
int tpAddCurrentPos(TP_STRUCT * const tp, EmcPose const * const disp)
{
    if (!tp || !disp) {
        return TP_ERR_MISSING_INPUT;
    }

    if (emcPoseValid(disp))
    {
        EmcPose old_pos = tp->currentPos;
        emcPoseSelfAdd(&tp->currentPos, disp);
        return TP_ERR_OK;
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR, "Tried to set invalid pose in tpAddCurrentPos on id %d!"
                "disp is %.12g, %.12g, %.12g\n",
                tp->execId,
                disp->tran.x,
                disp->tran.y,
                disp->tran.z);
        return TP_ERR_INVALID;
    }
}


/*
 * tpErrorCheck — 添加轨迹段前的有效性检查
 */
int tpErrorCheck(TP_STRUCT const * const tp) {

    if (!tp) {
        rtapi_print_msg(RTAPI_MSG_ERR, "TP is null\n");
        return TP_ERR_FAIL;
    }
    if (tp->aborting) {
        rtapi_print_msg(RTAPI_MSG_ERR, "TP is aborting\n");
        return TP_ERR_FAIL;
    }
    return TP_ERR_OK;
}


/*
 * tpCalculateTriangleVel — 计算三角速度轮廓的峰值速度
 */
static double tpCalculateTriangleVel(TC_STRUCT const *tc) {
    //Compute peak velocity for blend calculations
    double acc_scaled = tcGetTangentialMaxAccel(tc);
    double length = tc->target;
    if (!tc->finalized) {
        length /= 2.0;
    }
    return findVPeak(acc_scaled, length);
}

/*
 * tpCalculateOptimizationInitialVel — 计算优化初始速度
 */
static double tpCalculateOptimizationInitialVel(TP_STRUCT const * const tp, TC_STRUCT * const tc)
{
    double acc_scaled = tcGetTangentialMaxAccel(tc);
    double triangle_vel = findVPeak(acc_scaled, tc->target);
    double max_vel = tpGetMaxTargetVel(tp, tc);

    return fmin(triangle_vel, max_vel);
}

/*
 * tpInitBlendArcFromPrev — 从父子段初始化混合圆弧
 */
static int tpInitBlendArcFromPrev(TP_STRUCT const * const tp,
				  TC_STRUCT const * const prev_tc,
				  TC_STRUCT* const blend_tc,
				  double vel,
				  double ini_maxvel,
				  double acc)
{

#ifdef TP_SHOW_BLENDS
    int canon_motion_type = EMC_MOTION_TYPE_ARC;
#else
    int canon_motion_type = prev_tc->canon_motion_type;
#endif

    tcInit(blend_tc,
            TC_SPHERICAL,
            canon_motion_type,
            tp->cycleTime,
            prev_tc->enables,
            false);

    tcSetupState(blend_tc, tp);

    tcSetupMotion(blend_tc,
            vel,
            ini_maxvel,
            acc);

    double length;
    arcLength(&blend_tc->coords.arc.xyz, &length);
    blend_tc->target = length;
    blend_tc->nominal_length = length;

    tcSetTermCond(blend_tc, NULL, TC_TERM_COND_TANGENT);
    tcFinalizeLength(blend_tc);

    return TP_ERR_OK;
}

/*
 * tcSetLineXYZ — 用新的直线几何替换 TC 中的 XYZ
 */
static int tcSetLineXYZ(TC_STRUCT * const tc, PmCartLine const * const line)
{
    if (!line || tc->motion_type != TC_LINEAR) {
        return TP_ERR_FAIL;
    }
    if (!tc->coords.line.abc.tmag_zero || !tc->coords.line.uvw.tmag_zero) {
        rtapi_print_msg(RTAPI_MSG_ERR, "SetLineXYZ does not supportABC or UVW motion\n");
        return TP_ERR_FAIL;
    }

    tc->coords.line.xyz = *line;
    tc->target = line->tmag;
    return TP_ERR_OK;
}


/*
 * find_max_element — 找到数组中的最大元素索引
 */
static inline int find_max_element(double arr[], int sz)
{
    if (sz < 1) {
        return -1;
    }
    int max_idx = 0;
    int idx;
    for (idx = 0; idx < sz; ++idx) {
        if (arr[idx] > arr[max_idx]) {
            max_idx = idx;
        }
    }
    return max_idx;
}

/*
 * tpChooseBestBlend — 选择最优混合模式
 */
static tc_blend_type_t tpChooseBestBlend(TP_STRUCT const * const tp,
        TC_STRUCT * const prev_tc,
        TC_STRUCT * const tc,
        TC_STRUCT * const blend_tc)
{
    if (!tc || !prev_tc) {
        return NO_BLEND;
    }

    switch  (prev_tc->term_cond)
    {
    case TC_TERM_COND_EXACT:
    case TC_TERM_COND_STOP:
        return NO_BLEND;
    }
    double perf_parabolic = estimateParabolicBlendPerformance(tp, prev_tc, tc) / 2.0;
    double perf_tangent = prev_tc->kink_vel;
    double perf_arc_blend = blend_tc ? blend_tc->maxvel : 0.0;

    double perf[3] = {perf_parabolic, perf_tangent, perf_arc_blend};
    tc_blend_type_t best_blend = find_max_element(perf, 3);

    switch (best_blend) {
        case PARABOLIC_BLEND: // parabolic
            tcRemoveKinkProperties(prev_tc, tc);
            tcSetTermCond(prev_tc, tc, TC_TERM_COND_PARABOLIC);
            break;
        case TANGENT_SEGMENTS_BLEND: // tangent
            tcSetTermCond(prev_tc, tc, TC_TERM_COND_TANGENT);
            break;
        case ARC_BLEND: // arc blend
            tcRemoveKinkProperties(prev_tc, tc);

            break;
        case NO_BLEND:
            break;
    }
    return best_blend;
}


/*
 * tpCreateLineArcBlend — 创建直线-圆弧混合
 * 【执行步骤】
 *   1. 获取机器加速度/速度限制
 *   2. 初始化混合几何结构（blendInit3FromLineArc）
 *   3. 共面性检查（圆弧法向量与几何体法向量对齐）
 *   4. 计算混合参数（blendComputeParameters）
 *   5. 找到混合点（blendFindPoints3）
 *   6. 后处理得到精确点（blendLineArcPostProcess）
 *   7. 缩短前一段和后一段
 *   8. 创建混合圆弧（arcFromBlendPoints3）
 *   9. 选择最优混合模式
 *   10. 连接或消费段
 */
static tp_err_t tpCreateLineArcBlend(TP_STRUCT * const tp, TC_STRUCT * const prev_tc, TC_STRUCT * const tc, TC_STRUCT * const blend_tc)
{

    PmCartesian acc_bound, vel_bound;
    tpGetMachineAccelBounds(&acc_bound);
    tpGetMachineVelBounds(&vel_bound);

    BlendGeom3 geom;
    BlendParameters param;
    BlendPoints3 points_approx;
    BlendPoints3 points_exact;

    int res_init = blendInit3FromLineArc(&geom, &param,
            prev_tc,
            tc,
            &acc_bound,
            &vel_bound,
            emcmotConfig->maxAxisScale);

    if (res_init != TP_ERR_OK) {

        return res_init;
    }

    int coplanar = pmUnitCartsColinear(&geom.binormal,
            &tc->coords.circle.xyz.normal);

    if (!coplanar) {

        return TP_ERR_FAIL;
    }

    int res_param = blendComputeParameters(&param);

    int res_points = blendFindPoints3(&points_approx, &geom, &param);

    int res_post = blendLineArcPostProcess(&points_exact,
            &points_approx,
            &param,
            &geom, &prev_tc->coords.line.xyz,
            &tc->coords.circle.xyz);

    if (res_init || res_param || res_points || res_post)
    {
        return TP_ERR_FAIL;
    }

    if (points_exact.trim2 > param.phi2_max) {

        return TP_ERR_FAIL;
    }

    blendCheckConsume(&param, &points_exact, prev_tc, emcmotConfig->arcBlendGapCycles);
    PmCartLine line1_temp = prev_tc->coords.line.xyz;
    PmCircle circ2_temp = tc->coords.circle.xyz;

    double new_len1 = line1_temp.tmag - points_exact.trim1;
    int res_stretch1 = pmCartLineStretch(&line1_temp,
            new_len1,
            false);

    double phi2_new = tc->coords.circle.xyz.angle - points_exact.trim2;


    int res_stretch2 = pmCircleStretch(&circ2_temp,
            phi2_new,
            true);
    if (res_stretch1 || res_stretch2) {

        return TP_ERR_FAIL;
    }
    pmCartLinePoint(&line1_temp,
            line1_temp.tmag,
            &points_exact.arc_start);
    pmCirclePoint(&circ2_temp,
            0.0,
            &points_exact.arc_end);

    blendPoints3Print(&points_exact);
    int res_arc = arcFromBlendPoints3(&blend_tc->coords.arc.xyz,
            &points_exact,
            &geom,
            &param);
    if (res_arc < 0) {
        return TP_ERR_FAIL;
    }
    blend_tc->coords.arc.abc = prev_tc->coords.line.abc.end;
    blend_tc->coords.arc.uvw = prev_tc->coords.line.uvw.end;

    tpInitBlendArcFromPrev(tp, prev_tc, blend_tc, param.v_req,
            param.v_plan, param.a_max);

    int res_tangent = checkTangentAngle(&circ2_temp,
            &blend_tc->coords.arc.xyz,
            &geom,
            &param,
            tp->cycleTime,
            true);

    if (res_tangent < 0) {
        return TP_ERR_FAIL;
    }

    if (tpChooseBestBlend(tp, prev_tc, tc, blend_tc) != ARC_BLEND) {
        return TP_ERR_NO_ACTION;
    }

    if (param.consume) {
        int res_pop = tcqPopBack(&tp->queue);
        if (res_pop) {
            return TP_ERR_FAIL;
        }
    } else {
        tcSetLineXYZ(prev_tc, &line1_temp);
        blend_tc->atspeed=0;
    }
    tcSetCircleXYZ(tc, &circ2_temp);

    tcSetTermCond(prev_tc, tc, TC_TERM_COND_TANGENT);

    return TP_ERR_OK;
}


/*
 * tpCreateArcLineBlend — 创建圆弧-直线混合
 */
static tp_err_t tpCreateArcLineBlend(TP_STRUCT * const tp, TC_STRUCT * const prev_tc, TC_STRUCT * const tc, TC_STRUCT * const blend_tc)
{


    PmCartesian acc_bound, vel_bound;
    tpGetMachineAccelBounds(&acc_bound);
    tpGetMachineVelBounds(&vel_bound);

    BlendGeom3 geom;
    BlendParameters param;
    BlendPoints3 points_approx;
    BlendPoints3 points_exact;
    param.consume = 0;

    int res_init = blendInit3FromArcLine(&geom, &param,
            prev_tc,
            tc,
            &acc_bound,
            &vel_bound,
            emcmotConfig->maxAxisScale);
    if (res_init != TP_ERR_OK) {

        return res_init;
    }

    int coplanar = pmUnitCartsColinear(&geom.binormal,
            &prev_tc->coords.circle.xyz.normal);

    if (!coplanar) {

        return TP_ERR_FAIL;
    }

    int res_param = blendComputeParameters(&param);

    int res_points = blendFindPoints3(&points_approx, &geom, &param);

    int res_post = blendArcLinePostProcess(&points_exact,
            &points_approx,
            &param,
            &geom, &prev_tc->coords.circle.xyz,
            &tc->coords.line.xyz);

    if (res_init || res_param || res_points || res_post) {

        return TP_ERR_FAIL;
    }

    blendCheckConsume(&param, &points_exact, prev_tc, emcmotConfig->arcBlendGapCycles);

    PmCircle circ1_temp = prev_tc->coords.circle.xyz;
    PmCartLine line2_temp = tc->coords.line.xyz;

    double phi1_new = circ1_temp.angle - points_exact.trim1;

    if (points_exact.trim1 > param.phi1_max) {

        return TP_ERR_FAIL;
    }

    int res_stretch1 = pmCircleStretch(&circ1_temp,
            phi1_new,
            false);
    if (res_stretch1 != TP_ERR_OK) {
        return TP_ERR_FAIL;
    }

    double new_len2 = tc->target - points_exact.trim2;
    int res_stretch2 = pmCartLineStretch(&line2_temp,
            new_len2,
            true);

    if (res_stretch1 || res_stretch2) {
        return TP_ERR_FAIL;
    }

    pmCirclePoint(&circ1_temp,
            circ1_temp.angle,
            &points_exact.arc_start);

    pmCartLinePoint(&line2_temp,
            0.0,
            &points_exact.arc_end);

    blendPoints3Print(&points_exact);

    int res_arc = arcFromBlendPoints3(&blend_tc->coords.arc.xyz, &points_exact, &geom, &param);
    if (res_arc < 0) {
        return TP_ERR_FAIL;
    }

    blend_tc->coords.arc.abc = tc->coords.line.abc.start;
    blend_tc->coords.arc.uvw = tc->coords.line.uvw.start;

    tpInitBlendArcFromPrev(tp, prev_tc, blend_tc, param.v_req,
            param.v_plan, param.a_max);

    int res_tangent = checkTangentAngle(&circ1_temp, &blend_tc->coords.arc.xyz, &geom, &param, tp->cycleTime, false);
    if (res_tangent) {
        return TP_ERR_FAIL;
    }

    if (tpChooseBestBlend(tp, prev_tc, tc, blend_tc) != ARC_BLEND) {
        return TP_ERR_NO_ACTION;
    }

    tcSetCircleXYZ(prev_tc, &circ1_temp);
    tcSetLineXYZ(tc, &line2_temp);

    tc->blend_prev = 0;
    blend_tc->atspeed=0;
    tcSetTermCond(prev_tc, tc, TC_TERM_COND_TANGENT);
    return TP_ERR_OK;
}

/*
 * tpCreateArcArcBlend — 创建圆弧-圆弧混合
 */
static tp_err_t tpCreateArcArcBlend(TP_STRUCT * const tp, TC_STRUCT * const prev_tc, TC_STRUCT * const tc, TC_STRUCT * const blend_tc)
{
    int colinear = pmUnitCartsColinear(&prev_tc->coords.circle.xyz.normal,
            &tc->coords.circle.xyz.normal);
    if (!colinear) {
        // Fail out if not collinear
        return TP_ERR_FAIL;
    }

    PmCartesian acc_bound, vel_bound;
    tpGetMachineAccelBounds(&acc_bound);
    tpGetMachineVelBounds(&vel_bound);

    BlendGeom3 geom;
    BlendParameters param;
    BlendPoints3 points_approx;
    BlendPoints3 points_exact;

    int res_init = blendInit3FromArcArc(&geom, &param,
            prev_tc,
            tc,
            &acc_bound,
            &vel_bound,
            emcmotConfig->maxAxisScale);

    if (res_init != TP_ERR_OK) {
        return res_init;
    }

    int coplanar1 = pmUnitCartsColinear(&geom.binormal,
            &prev_tc->coords.circle.xyz.normal);

    if (!coplanar1) {
        return TP_ERR_FAIL;
    }

    int coplanar2 = pmUnitCartsColinear(&geom.binormal,
            &tc->coords.circle.xyz.normal);
    if (!coplanar2) {
        return TP_ERR_FAIL;
    }

    int res_param = blendComputeParameters(&param);
    int res_points = blendFindPoints3(&points_approx, &geom, &param);

    int res_post = blendArcArcPostProcess(&points_exact,
            &points_approx,
            &param,
            &geom, &prev_tc->coords.circle.xyz,
            &tc->coords.circle.xyz);

    if (res_init || res_param || res_points || res_post) {

        return TP_ERR_FAIL;
    }

    blendCheckConsume(&param, &points_exact, prev_tc, emcmotConfig->arcBlendGapCycles);

    double phi1_new = prev_tc->coords.circle.xyz.angle - points_exact.trim1;
    double phi2_new = tc->coords.circle.xyz.angle - points_exact.trim2;


    if (points_exact.trim1 > param.phi1_max) {

        return TP_ERR_FAIL;
    }

    if (points_exact.trim2 > param.phi2_max) {

        return TP_ERR_FAIL;
    }

    PmCircle circ1_temp = prev_tc->coords.circle.xyz;
    PmCircle circ2_temp = tc->coords.circle.xyz;

    int res_stretch1 = pmCircleStretch(&circ1_temp,
            phi1_new,
            false);
    if (res_stretch1 != TP_ERR_OK) {
        return TP_ERR_FAIL;
    }

    int res_stretch2 = pmCircleStretch(&circ2_temp,
            phi2_new,
            true);
    if (res_stretch1 || res_stretch2) {
        return TP_ERR_FAIL;
    }

    pmCirclePoint(&circ1_temp,
            circ1_temp.angle,
            &points_exact.arc_start);
    pmCirclePoint(&circ2_temp,
            0.0,
            &points_exact.arc_end);


    blendPoints3Print(&points_exact);
    int res_arc = arcFromBlendPoints3(&blend_tc->coords.arc.xyz, &points_exact, &geom, &param);
    if (res_arc < 0) {
        return TP_ERR_FAIL;
    }

    blend_tc->coords.arc.abc = prev_tc->coords.circle.abc.end;
    blend_tc->coords.arc.uvw = prev_tc->coords.circle.uvw.end;

    tpInitBlendArcFromPrev(tp, prev_tc, blend_tc, param.v_req,
            param.v_plan, param.a_max);

    int res_tangent1 = checkTangentAngle(&circ1_temp, &blend_tc->coords.arc.xyz, &geom, &param, tp->cycleTime, false);
    int res_tangent2 = checkTangentAngle(&circ2_temp, &blend_tc->coords.arc.xyz, &geom, &param, tp->cycleTime, true);
    if (res_tangent1 || res_tangent2) {
        return TP_ERR_FAIL;
    }

    if (tpChooseBestBlend(tp, prev_tc, tc, blend_tc) != ARC_BLEND) {
        return TP_ERR_NO_ACTION;
    }

    tcSetCircleXYZ(prev_tc, &circ1_temp);
    tcSetCircleXYZ(tc, &circ2_temp);

    tc->blend_prev = 0;
    blend_tc->atspeed=0;
    tcSetTermCond(prev_tc, tc, TC_TERM_COND_TANGENT);
    return TP_ERR_OK;
}


/*
 * tpCreateLineLineBlend — 创建直线-直线混合
 */
static tp_err_t tpCreateLineLineBlend(TP_STRUCT * const tp, TC_STRUCT * const prev_tc,
        TC_STRUCT * const tc, TC_STRUCT * const blend_tc)
{

    PmCartesian acc_bound, vel_bound;

    tpGetMachineAccelBounds(&acc_bound);
    tpGetMachineVelBounds(&vel_bound);

    BlendGeom3 geom;
    BlendParameters param;
    BlendPoints3 points;

    int res_init = blendInit3FromLineLine(&geom, &param,
            prev_tc,
            tc,
            &acc_bound,
            &vel_bound,
            emcmotConfig->maxAxisScale);

    if (res_init != TP_ERR_OK) {

        return res_init;
    }

    int res_blend = blendComputeParameters(&param);
    if (res_blend != TP_ERR_OK) {
        return res_blend;
    }

    blendFindPoints3(&points, &geom, &param);

    blendCheckConsume(&param, &points, prev_tc, emcmotConfig->arcBlendGapCycles);

    int res_arc = arcFromBlendPoints3(&blend_tc->coords.arc.xyz, &points, &geom, &param);
    if (res_arc < 0) {
        return TP_ERR_FAIL;
    }

    blend_tc->coords.arc.abc = prev_tc->coords.line.abc.end;
    blend_tc->coords.arc.uvw = prev_tc->coords.line.uvw.end;

    tpInitBlendArcFromPrev(tp, prev_tc, blend_tc, param.v_req,
            param.v_plan, param.a_max);


    if (tpChooseBestBlend(tp, prev_tc, tc, blend_tc) != ARC_BLEND) {
        return TP_ERR_NO_ACTION;
    }

    int retval = TP_ERR_FAIL;

    if (param.consume) {
        retval = tcqPopBack(&tp->queue);
        if (retval) {
            rtapi_print_msg(RTAPI_MSG_ERR, "PopBack failed\n");
            return TP_ERR_FAIL;
        }
        retval = tcConnectBlendArc(NULL, tc, &points.arc_start, &points.arc_end);
    } else {
        retval = tcConnectBlendArc(prev_tc, tc, &points.arc_start, &points.arc_end);
        blend_tc->atspeed=0;
    }
    return retval;
}


/**
 * tpAddSegmentToQueue — 将轨迹段加入队列
 */
static inline int tpAddSegmentToQueue(TP_STRUCT * const tp, TC_STRUCT * const tc, int inc_id) {

    /* 为段分配 ID */
    tc->id = tp->nextId;
    if (tcqPut(&tp->queue, tc) == -1) {
        rtapi_print_msg(RTAPI_MSG_ERR, "tcqPut failed.\n");
        return TP_ERR_FAIL;
    }
    /* 是否递增下一个 ID */
    if (inc_id) {
        tp->nextId++;
    }

    //更新规划器目标位置
    /* 刚性攻丝的目标位置是延伸的（包括 overrun），所以不更新 goalPos */
    if (tc->motion_type != TC_RIGIDTAP) {
        tcGetEndpoint(tc, &tp->goalPos);
    }
    tp->done = 0;
    tp->depth = tcqLen(&tp->queue);

    return TP_ERR_OK;
}

/*
 * handleModeChange — 处理运动模式变化
 */
static int handleModeChange(TC_STRUCT * const prev_tc, TC_STRUCT * const tc)
{
    if (!tc || !prev_tc) {
        return TP_ERR_FAIL;
    }
    if ((prev_tc->canon_motion_type == 1) ^
            (tc->canon_motion_type == 1)) {

        tcSetTermCond(prev_tc, tc, TC_TERM_COND_STOP);
    }
    if (prev_tc->synchronized != TC_SYNC_POSITION &&
            tc->synchronized == TC_SYNC_POSITION) {

        tcSetTermCond(prev_tc, tc, TC_TERM_COND_STOP);
    }
    return TP_ERR_OK;
}

/*
 * tpCheckBlendArcType — 检查可以创建哪种混合圆弧
 */
static blend_type_t tpCheckBlendArcType(
        TC_STRUCT const * const prev_tc,
        TC_STRUCT const * const tc) {

    if (!prev_tc || !tc) {
        return BLEND_NONE;
    }

    if (prev_tc->term_cond != TC_TERM_COND_PARABOLIC) {
        return BLEND_NONE;
    }

    if (tcRotaryMotionCheck(tc) || tcRotaryMotionCheck(prev_tc)) {
        return BLEND_NONE;
    }

    if (tc->finalized || prev_tc->finalized) {
        return BLEND_NONE;
    }

    if ((prev_tc->motion_type == TC_LINEAR) && (tc->motion_type == TC_LINEAR)) {
        return BLEND_LINE_LINE;
    } else if (prev_tc->motion_type == TC_LINEAR && tc->motion_type == TC_CIRCULAR) {
        return BLEND_LINE_ARC;
    } else if (prev_tc->motion_type == TC_CIRCULAR && tc->motion_type == TC_LINEAR) {
        return BLEND_ARC_LINE;
    } else if (prev_tc->motion_type == TC_CIRCULAR && tc->motion_type == TC_CIRCULAR) {
        return BLEND_ARC_ARC;
    } else {
        return BLEND_NONE;
    }
}


/*
 * tpComputeOptimalVelocity — 计算最优最终速度
 */
static int tpComputeOptimalVelocity(TP_STRUCT const * const tp, TC_STRUCT * const tc, TC_STRUCT * const prev1_tc) {
    //// 获取当前段最大切向加速度
    double acc_this = tcGetTangentialMaxAccel(tc);

    double vs_back = pmSqrt(pmSq(tc->finalvel) + 2.0 * acc_this * tc->target);

    // 获取两段各自的速度上限
    double vf_limit_this = tc->maxvel;
    double vf_limit_prev = prev1_tc->maxvel;
    if (prev1_tc->kink_vel >=0  && prev1_tc->term_cond == TC_TERM_COND_TANGENT)
    {
        vf_limit_prev = fmin(vf_limit_prev, prev1_tc->kink_vel);
    }
    double vf_limit = fmin(vf_limit_this, vf_limit_prev);

    if (vs_back >= vf_limit ) {
        vs_back = vf_limit;
        prev1_tc->optimization_state = TC_OPTIM_AT_MAX;
    }

    prev1_tc->finalvel = vs_back;

    double sample_maxvel = tc->target / (tp->cycleTime * TP_MIN_SEGMENT_CYCLES);
    tc->maxvel = fmin(tc->maxvel, sample_maxvel);

    return TP_ERR_OK;
}


/*
 * tpRunOptimization — 速度优化（"涨潮"算法）
 */
static int tpRunOptimization(TP_STRUCT * const tp)
{
   TC_STRUCT *tc;
    TC_STRUCT *prev1_tc;

    int ind, x;
    int len = tcqLen(&tp->queue);
    //TODO make lookahead depth configurable from the INI file

    int hit_peaks = 0;
    bool hit_non_tangent = false;

    //确定轨迹队列中每个线段允许的最终速度
    for (x = 1; x < emcmotConfig->arcBlendOptDepth + 2; ++x)
    {
        ind = len-x;
        tc = tcqItem(&tp->queue, ind);             // 当前轨迹段
        prev1_tc = tcqItem(&tp->queue, ind-1);   // 前序轨迹段

        if ( !prev1_tc || !tc) {
            return TP_ERR_OK;
        }

        //非切线轨迹（如90°拐角）必须降速
        if (prev1_tc->term_cond != TC_TERM_COND_TANGENT) {
            if (hit_non_tangent) {
                return TP_ERR_OK;
            } else  {
                hit_non_tangent = true;
                continue;
            }
        }

        //已执行过半的轨迹段不再优化
        double progress_ratio = prev1_tc->progress / prev1_tc->target;
        double cutoff_ratio = BLEND_DIST_FRACTION / 2.0;

        if (progress_ratio >= cutoff_ratio) {
            return TP_ERR_OK;
        }

        if (prev1_tc->splitting || prev1_tc->blending_next) {
            return TP_ERR_OK;
        }

        if (tc->atspeed) {
            tc->finalvel = 0.0;
        }

        if (!tc->finalized) {
            prev1_tc->finalvel = fmin(prev1_tc->maxvel, tpCalculateOptimizationInitialVel(tp,tc));

            // 速度拐点限制
            if (prev1_tc->kink_vel >=0  && prev1_tc->term_cond == TC_TERM_COND_TANGENT) {
              prev1_tc->finalvel = fmin(prev1_tc->finalvel, prev1_tc->kink_vel);
            }
            tc->finalvel = 0.0;
        } else {
            tpComputeOptimalVelocity(tp, tc, prev1_tc);
        }

        tc->active_depth = x - 2 - hit_peaks;

        //惰性优化
        if (tc->optimization_state == TC_OPTIM_AT_MAX) {
            hit_peaks++;
        }
        if (hit_peaks > TP_OPTIMIZATION_CUTOFF) {
            return TP_ERR_OK;
        }
    }
    return TP_ERR_OK;
}


/*
 * tpSetupTangent — 设置切向模式
 * 检查当前段和前一段是否相切，如果是则设置切向模式
 */
static int tpSetupTangent(TP_STRUCT const * const tp,
        TC_STRUCT * const prev_tc, TC_STRUCT * const tc) {
    if (!tc || !prev_tc) {
        return TP_ERR_FAIL;
    }
    if (tcRotaryMotionCheck(tc) || tcRotaryMotionCheck(prev_tc)) {
        return TP_ERR_FAIL;
    }

    if (emcmotConfig->arcBlendOptDepth < 2) {
        return TP_ERR_FAIL;
    }

    if (prev_tc->term_cond == TC_TERM_COND_STOP) {
        return TP_ERR_FAIL;
    }

    PmCartesian prev_tan, this_tan;

    int res_endtan = tcGetEndTangentUnitVector(prev_tc, &prev_tan);
    int res_starttan = tcGetStartTangentUnitVector(tc, &this_tan);
    if (res_endtan || res_starttan) {
    }

    const double SHARP_CORNER_DEG = 2.0;
    const double SHARP_CORNER_EPSILON = pmSq(PM_PI * ( SHARP_CORNER_DEG / 180.0));
    if (pmCartCartAntiParallel(&prev_tan, &this_tan, SHARP_CORNER_EPSILON))
    {
        tcSetTermCond(prev_tc, tc, TC_TERM_COND_STOP);
        return TP_ERR_FAIL;
    }

    double v_max1 = tcGetMaxTargetVel(prev_tc, getMaxFeedScale(prev_tc));
    double v_max2 = tcGetMaxTargetVel(tc, getMaxFeedScale(tc));
    double v_max = fmin(v_max1, v_max2);

    double a_inst = v_max / tp->cycleTime + tc->maxaccel;
    PmCartesian acc1, acc2, acc_diff;
    pmCartScalMult(&prev_tan, a_inst, &acc1);
    pmCartScalMult(&this_tan, a_inst, &acc2);
    pmCartCartSub(&acc2,&acc1,&acc_diff);

    PmCartesian acc_bound;
    tpGetMachineAccelBounds(&acc_bound);

    PmCartesian acc_scale;
    findAccelScale(&acc_diff,&acc_bound,&acc_scale);

    double acc_scale_max = pmCartAbsMax(&acc_scale);
    if (prev_tc->motion_type == TC_CIRCULAR || tc->motion_type == TC_CIRCULAR) {
        acc_scale_max /= BLEND_ACC_RATIO_TANGENTIAL;
    }

    const double kink_ratio = tpGetTangentKinkRatio();

    if (acc_scale_max < kink_ratio) {
        tcSetTermCond(prev_tc, tc, TC_TERM_COND_TANGENT);
        tcSetKinkProperties(prev_tc, tc, v_max, acc_scale_max);
        return TP_ERR_OK;
    } else {
        tcSetKinkProperties(prev_tc, tc, v_max * kink_ratio / acc_scale_max, kink_ratio);

        return TP_ERR_NO_ACTION;
    }
}

/*
 * tpCreateBlendIfPossible — 如果可能则创建混合圆弧
 */
static bool tpCreateBlendIfPossible(
        TP_STRUCT *tp,
        TC_STRUCT *prev_tc,
        TC_STRUCT *tc,
        TC_STRUCT *blend_tc)
{
    tp_err_t res_create = TP_ERR_FAIL;
    blend_type_t blend_requested = tpCheckBlendArcType(prev_tc, tc);

    switch (blend_requested) {
        case BLEND_LINE_LINE:
            res_create = tpCreateLineLineBlend(tp, prev_tc, tc, blend_tc);
            break;
        case BLEND_LINE_ARC:
            res_create = tpCreateLineArcBlend(tp, prev_tc, tc, blend_tc);
            break;
        case BLEND_ARC_LINE:
            res_create = tpCreateArcLineBlend(tp, prev_tc, tc, blend_tc);
            break;
        case BLEND_ARC_ARC:
            res_create = tpCreateArcArcBlend(tp, prev_tc, tc, blend_tc);
            break;
        case BLEND_NONE:
        default:
            res_create = TP_ERR_FAIL;
            break;
    }

    return res_create == TP_ERR_OK;
}


/*
 * tpHandleBlendArc — 处理混合圆弧创建
 * 处理创建新混合圆弧的检查、设置和计算
 */
static tc_blend_type_t tpHandleBlendArc(TP_STRUCT * const tp, TC_STRUCT * const tc) {

    TC_STRUCT *prev_tc;
    prev_tc = tcqLast(&tp->queue);

    if ( !prev_tc) {
        return NO_BLEND;
    }
    /* 前一段已执行过半，不能再混合 */
    if (prev_tc->progress > prev_tc->target / 2.0) {
        return NO_BLEND;
    }

    int res_tan = tpSetupTangent(tp, prev_tc, tc);
    switch (res_tan) {
        case TP_ERR_FAIL:

        case TP_ERR_OK:
            return res_tan;
        case TP_ERR_NO_ACTION:
        default:
            break;
    }

    TC_STRUCT blend_tc = {0};

    tc_blend_type_t blend_used = NO_BLEND;

    bool arc_blend_ok = tpCreateBlendIfPossible(tp, prev_tc, tc, &blend_tc);

    if (arc_blend_ok) {
        blend_used = ARC_BLEND;
        tpAddSegmentToQueue(tp, &blend_tc, false);
    } else {
        blend_used = tpChooseBestBlend(tp, prev_tc, tc, NULL) ;
    }

    return blend_used;
}

/*
 * tpAddLine — 添加直线轨迹段
 * 【参数】
 *   - tp                  : 轨迹规划器指针
 *   - end                 : 目标位姿（XYZABCUVW）
 *   - canon_motion_type   : 解释器运动类型（traverse/feed/arc）
 *   - vel                 : 请求速度（F word）
 *   - ini_maxvel          : INI 文件允许的最大速度
 *   - acc                 : 最大加速度
 *   - enables             : 使能位（进给倍率等）
 *   - atspeed            : 主轴到位标志
 *   - indexer_jnum       : 索引旋转关节编号
 *
 * 【执行步骤】
 *   1. 错误检查（tpErrorCheck）
 *   2. 初始化 TC 结构体（tcInit）
 *   3. 设置状态（tcSetupState）
 *   4. 设置运动参数（tcSetupMotion）
 *   5. 初始化直线几何（pmLine9Init）
 *   6. 计算目标距离（pmLine9Target）
 *   7. 零长度检查
 *   8. 速度限制（tcClampVelocityByLength）
 *   9. 处理模式变化（handleModeChange）
 *   10. 处理混合圆弧（tpHandleBlendArc）
 *   11. 定稿前一段（tcFinalizeLength）
 *   12. 标记提前停止（tcFlagEarlyStop）
 *   13. 加入队列（tpAddSegmentToQueue）
 *   14. 运行速度优化（tpRunOptimization）
 */
int tpAddLine(TP_STRUCT * const tp, EmcPose end, int canon_motion_type,
            double vel, double ini_maxvel, double acc, unsigned char enables,
            char atspeed, int indexer_jnum)
{
    if (tpErrorCheck(tp) < 0)
    {
        return TP_ERR_FAIL;
    }

    //初始化轨迹规划器
    TC_STRUCT tc = {0};
    tcInit(&tc,TC_LINEAR,canon_motion_type,tp->cycleTime,enables,atspeed);

    // 从轨迹规划器复制状态数据
    tcSetupState(&tc, tp);

    // 复制运动参数
    tcSetupMotion(&tc,vel,ini_maxvel,acc);
    //将一个 9 轴笛卡尔直线运动（Line9）对象初始化为从起点 start 到终点 end 的直线段，同时为 XYZ、ABC、UVW 三组坐标分别计算方向向量和长度信息
    pmLine9Init(&tc.coords.line,&tp->goalPos, &end);
    //计算起点到终点直线距离
    tc.target = pmLine9Target(&tc.coords.line);

    //零长度校验
    if (tc.target < TP_POS_EPSILON)
    {
        rtapi_print_msg(RTAPI_MSG_DBG,"failed to create line id %d, zero-length segment\n",tp->nextId);
        return TP_ERR_ZERO_LENGTH;
    }
    tc.nominal_length = tc.target;
    //速度限制约束
    tcClampVelocityByLength(&tc);

    //（对于锁定式分度器）要执行此动作，应解锁哪个关节，-1表示不解锁
    tc.indexer_jnum = indexer_jnum;

    //TODO refactor this into its own function
    TC_STRUCT *prev_tc;
    // 获取队列末尾的上一线段
    prev_tc = tcqLast(&tp->queue);
    // 检查运动模式变化
    handleModeChange(prev_tc, &tc);
    if (emcmotConfig->arcBlendEnable){
        // 若启用圆弧过渡，计算混合路径
        tpHandleBlendArc(tp, &tc);
    }
    tcFinalizeLength(prev_tc);
    // 最终确定上一线段长度
    tcFlagEarlyStop(prev_tc, &tc);

    //将新线段加入规划队列，并触发速度优化
    int retval = tpAddSegmentToQueue(tp, &tc, true);
    tpRunOptimization(tp);

    tc.finalvel = vel;

    return retval;
}


/**
 * tpAddCircle — 添加圆弧轨迹段
 */
int tpAddCircle(TP_STRUCT * const tp,
        EmcPose end,
        PmCartesian center,
        PmCartesian normal,
        int turn,
        int canon_motion_type,
        double vel,
        double ini_maxvel,
        double acc,
        unsigned char enables,
        char atspeed)
{
    if (tpErrorCheck(tp)<0) {
        return TP_ERR_FAIL;
    }

    TC_STRUCT tc = {0};

    //初始化规划器
    tcInit(&tc,
            TC_CIRCULAR,
            canon_motion_type,
            tp->cycleTime,
            enables,
            atspeed);


    //状态更新
    tcSetupState(&tc, tp);

    //五参数定义法：起点（规划器当前位置）、终点、圆心、法向量、整圆圈数
    int res_init = pmCircle9Init(&tc.coords.circle,
            &tp->goalPos,
            &end,
            &center,
            &normal,
            turn);

    if (res_init) return res_init;

    tc.target = pmCircle9Target(&tc.coords.circle); // 计算实际弧长
    if (tc.target < TP_POS_EPSILON) {
        return TP_ERR_ZERO_LENGTH;
    }
    tc.nominal_length = tc.target;

    tcSetupMotion(&tc,
            vel,
            ini_maxvel,
            acc);

    tcClampVelocityByLength(&tc);  // 基于弧长限制速度

    TC_STRUCT *prev_tc;
    prev_tc = tcqLast(&tp->queue);

    handleModeChange(prev_tc, &tc);
    if (emcmotConfig->arcBlendEnable){
        tpHandleBlendArc(tp, &tc);
        findSpiralArcLengthFit(&tc.coords.circle.xyz, &tc.coords.circle.fit);
    }
    tcFinalizeLength(prev_tc);
    tcFlagEarlyStop(prev_tc, &tc);// 标记提前减速

    int retval = tpAddSegmentToQueue(tp, &tc, true);

    tpRunOptimization(tp);
    return retval;
}


/*
 * tpComputeBlendVelocity — 计算混合速度
 *计算两个轨迹段交接处的平滑衔接速度
 *不超过各自的最大加速度；
 *不超过每段的可达最高速度；
 *满足路径几何衔接角度与轨迹偏差（tolerance）约束；
 *
 * 【算法步骤】
 *   1. 计算每段的可达速度（三角轮廓峰值和目标速度中的较小值）
 *   2. 计算每段的最大允许混合时间
 *   3. 计算混合时间（取最小）
 *   4. 根据混合时间计算混合速度
 *   5. 考虑容差约束进一步限制速度
 *   6. 计算净切向速度
 */
static int tpComputeBlendVelocity(
        TC_STRUCT const *tc,
        TC_STRUCT const *nexttc,
        double target_vel_this,
        double target_vel_next,
        double *v_blend_this,
        double *v_blend_next,
        double *v_blend_net)
{
    /* Pre-checks for valid pointers */
    if (!nexttc || !tc || !v_blend_this || !v_blend_next ) {
        return TP_ERR_FAIL;
    }

    //提取每段最大加速度
    double acc_this = tcGetTangentialMaxAccel(tc);
    double acc_next = tcGetTangentialMaxAccel(nexttc);

    //计算可达速度
    double v_reachable_this = fmin(tpCalculateTriangleVel(tc), target_vel_this);
    double v_reachable_next = fmin(tpCalculateTriangleVel(nexttc), target_vel_next);

    //计算每段的最大允许时间
    double t_max_this = tc->target / v_reachable_this;
    double t_max_next = nexttc->target / v_reachable_next;
    double t_max_reachable = fmin(t_max_this, t_max_next);

    // How long the blend phase would be at maximum acceleration
    //计算blend时间上下限并确定最终blend时间
    double t_min_blend_this = v_reachable_this / acc_this;
    double t_min_blend_next = v_reachable_next / acc_next;

    double t_max_blend = fmax(t_min_blend_this, t_min_blend_next);
    double t_blend = fmin(t_max_reachable, t_max_blend);

    //计算blend速度
    *v_blend_this = fmin(v_reachable_this, t_blend * acc_this);
    *v_blend_next = fmin(v_reachable_next, t_blend * acc_next);

    double theta;

    PmCartesian v1, v2;

    //角度修正
    tcGetEndAccelUnitVector(tc, &v1);
    tcGetStartAccelUnitVector(nexttc, &v2);
    findIntersectionAngle(&v1, &v2, &theta);

    double cos_theta = cos(theta);

    if (tc->tolerance > 0) {
        double tblend_vel;
        const double min_cos_theta = cos(PM_PI / 2.0 - TP_MIN_ARC_ANGLE);
        if (cos_theta > min_cos_theta) {
            tblend_vel = 2.0 * pmSqrt(acc_this * tc->tolerance / cos_theta);
            *v_blend_this = fmin(*v_blend_this, tblend_vel);
            *v_blend_next = fmin(*v_blend_next, tblend_vel);
        }
    }

    //计算净切向速度（合成方向）
    if (v_blend_net) {
        *v_blend_net = sin(theta) * (*v_blend_this + *v_blend_next) / 2.0;
    }

    return TP_ERR_OK;
}

/*
 * estimateParabolicBlendPerformance — 估算抛物线混合性能
 */
static double estimateParabolicBlendPerformance(
        TP_STRUCT const *tp,
        TC_STRUCT const *tc,
        TC_STRUCT const *nexttc)
{
    double v_this = 0.0, v_next = 0.0;

    double target_vel_this = tpGetMaxTargetVel(tp, tc);
    double target_vel_next = tpGetMaxTargetVel(tp, nexttc);

    double v_net = 0.0;
    tpComputeBlendVelocity(tc, nexttc, target_vel_this, target_vel_next, &v_this, &v_next, &v_net);

    return v_net;
}

/*
 * tcUpdateDistFromAccel — 根据加速度更新位移
 */
static int tcUpdateDistFromAccel(TC_STRUCT * const tc, double acc, double vel_desired, int reverse_run)
{
    double v_next = tc->currentvel + acc * tc->cycle_time;
    if (v_next < 0.0) {
        v_next = 0.0;
        //若剩余距离小于当前周期以当前速度能走的距离，则直接把 progress 设为目标位置，避免在末端的欠收或小幅摆动
        if (tcGetDistanceToGo(tc,reverse_run) < (tc->currentvel *  tc->cycle_time))
        {
            tc->progress = tcGetTarget(tc,reverse_run);
        }
    } else {
        //计算本周期位移增量
        double displacement = (v_next + tc->currentvel) * 0.5 * tc->cycle_time;

        double disp_sign = reverse_run ? -1 : 1;
        tc->progress += (disp_sign * displacement);

        //使用 bisaturate 保证 tc->progress 保持在合法目标区间内
        tc->progress = bisaturate(tc->progress, tcGetTarget(tc, TC_DIR_FORWARD), tcGetTarget(tc, TC_DIR_REVERSE));
    }

    tc->currentvel = v_next;

    tc->on_final_decel = (fabs(vel_desired - tc->currentvel) < TP_VEL_EPSILON) && (acc < 0.0);

    return TP_ERR_OK;
}

/*
 * tpDebugCycleInfo — 调试输出周期信息
 */
static void tpDebugCycleInfo(TP_STRUCT const * const tp, TC_STRUCT const * const tc, TC_STRUCT const * const nexttc, double acc) {
#ifdef TC_DEBUG
    double tc_target_vel = tpGetRealTargetVel(tp, tc);
    double tc_finalvel = tpGetRealFinalVel(tp, tc, nexttc);

    tc_debug_print("tc state: vr = %f, vf = %f, maxvel = %f\n",
            tc_target_vel, tc_finalvel, tc->maxvel);
    tc_debug_print("          currentvel = %f, fs = %f, tc = %f, term = %d\n",
            tc->currentvel, tpGetFeedScale(tp,tc), tc->cycle_time, tc->term_cond);
    tc_debug_print("          acc = %f, T = %f, DTG = %.12g\n", acc,
            tcGetTarget(tc,tp->reverse_run), tcGetDistanceToGo(tc,tp->reverse_run));
    tc_debug_print("          reverse_run = %d\n", tp->reverse_run);
    tc_debug_print("          motion type %d\n", tc->motion_type);

    if (tc->on_final_decel) {
        rtapi_print(" on final decel\n");
    }
#else
    (void)tp;
    (void)tc;
    (void)nexttc;
    (void)acc;
#endif
}

/*
 * tpCalculateTrapezoidalAccel — 计算梯形加速度
 * 根据梯形速度轮廓计算当前周期应使用的加速度和期望速度
 *
 * 【核心公式（判别式）】
 *   discr = vf² + 2*acc*(2*dx - vi*dt) + (acc*dt/2)²
 */
void tpCalculateTrapezoidalAccel(TP_STRUCT const * const tp, TC_STRUCT * const tc, TC_STRUCT const * const nexttc,
        double * const acc, double * const vel_desired)
{
    double tc_target_vel = tpGetRealTargetVel(tp, tc);
    double tc_finalvel = tpGetRealFinalVel(tp, tc, nexttc);
    double dx = tcGetDistanceToGo(tc, tp->reverse_run);
    double maxaccel = tcGetTangentialMaxAccel(tc);

    double discr_term1 = pmSq(tc_finalvel);
    double discr_term2 = maxaccel * (2.0 * dx - tc->currentvel * tc->cycle_time);
    double tmp_adt = maxaccel * tc->cycle_time * 0.5;
    double discr_term3 = pmSq(tmp_adt);

    double discr = discr_term1 + discr_term2 + discr_term3;

    if (discr < 0.0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "discriminant %f < 0 in velocity calculation!\n", discr);
    }

    double maxnewvel = -tmp_adt;

    if (discr > discr_term3) {
        maxnewvel += pmSqrt(discr);
    }

    double newvel = saturate(maxnewvel, tc_target_vel);

    double dt = fmax(tc->cycle_time, TP_TIME_EPSILON);

    double maxnewaccel = (newvel - tc->currentvel) / dt;

    *acc = saturate(maxnewaccel, maxaccel);
    *vel_desired = maxnewvel;
}

/*
 * tpCalculateRampAccel — 计算斜坡加速度
 */
static int tpCalculateRampAccel(TP_STRUCT const * const tp,
        TC_STRUCT * const tc,
        TC_STRUCT const * const nexttc,
        double * const acc,
        double * const vel_desired)
{
    //当前段（tc）剩余的位移
    double dx = tcGetDistanceToGo(tc, tp->reverse_run);

    if (!tc->blending_next) {
        tc->vel_at_blend_start = tc->currentvel;
    }

    //获取本段实际计划达到的最终速度
    double vel_final = tpGetRealFinalVel(tp, tc, nexttc);

    //当目标速度太小（接近 0）时，不能用速度"ramp"策略，上层会回退到 trapezoidal 策略
    if (vel_final < TP_VEL_EPSILON) {
        return TP_ERR_FAIL;
    }

    double vel_avg = (tc->currentvel + vel_final) / 2.0;

    double dt = 1e-16;
    if (vel_avg > TP_VEL_EPSILON) {
        dt = fmax( dx / vel_avg, 1e-16);
    }

    double dv = vel_final - tc->currentvel;

    double acc_final = dv / dt;

    double acc_max = tcGetTangentialMaxAccel(tc);

    *acc = saturate(acc_final, acc_max);
    *vel_desired = vel_final;

    return TP_ERR_OK;
}

/*
 * tpUpdateMovementStatus — 更新运动状态到共享内存
 */
static int tpUpdateMovementStatus(TP_STRUCT * const tp, TC_STRUCT const * const tc ) {


    if (!tp) {
        return TP_ERR_FAIL;
    }

    if (!tc) {
        emcmotStatus->distance_to_go = 0;
        emcmotStatus->enables_queued = emcmotStatus->enables_new;
        emcmotStatus->requested_vel = 0;
        emcmotStatus->current_vel = 0;

        emcPoseZero(&emcmotStatus->dtg);

        tp->motionType = 0;
        tp->activeDepth = 0;
        return TP_ERR_STOPPED;
    }

    EmcPose tc_pos;
    tcGetEndpoint(tc, &tc_pos);

    tp->motionType = tc->canon_motion_type;
    tp->activeDepth = tc->active_depth;
    emcmotStatus->distance_to_go = tc->target - tc->progress;
    emcmotStatus->enables_queued = tc->enables;
    tp->execId = tc->id;
    emcmotStatus->requested_vel = tc->reqvel;
    emcmotStatus->current_vel = tc->currentvel;

    emcPoseSub(&tc_pos, &tp->currentPos, &emcmotStatus->dtg);
    return TP_ERR_OK;
}


/*
 * tpUpdateBlend — 更新混合状态
 * 执行实际的混合过程：更新下一段的目标速度，然后运行周期更新
 */
static void tpUpdateBlend(TP_STRUCT * const tp, TC_STRUCT * const tc,
        TC_STRUCT * const nexttc) {

    if (!nexttc) {
        return;
    }
    double save_vel = nexttc->target_vel;

    if (tpGetFeedScale(tp, nexttc) > TP_VEL_EPSILON) {
        double dv = tc->vel_at_blend_start - tc->currentvel;
        double vel_start = fmax(tc->vel_at_blend_start, TP_VEL_EPSILON);
        double blend_progress = fmax(fmin(dv / vel_start, 1.0), 0.0);
        double blend_scale = tc->vel_at_blend_start / tc->blend_vel;
        nexttc->target_vel = blend_progress * nexttc->blend_vel * blend_scale;
        nexttc->is_blending = true;
    } else {
        nexttc->target_vel = 0.0;
    }

    tpUpdateCycle(tp, nexttc, NULL);
    nexttc->target_vel = save_vel;
}


/*
 * tpHandleEmptyQueue — 处理空队列
 * 如果程序结束或遇到队列饥饿，执行软复位
 */
static void tpHandleEmptyQueue(TP_STRUCT * const tp)
{

    tcqInit(&tp->queue);
    tp->goalPos = tp->currentPos;
    tp->done = 1;
    tp->depth = tp->activeDepth = 0;
    tp->aborting = 0;
    tp->execId = 0;
    tp->motionType = 0;

    tpUpdateMovementStatus(tp, NULL);

    tpResume(tp);
}

/** tpSetRotaryUnlock — 解锁旋转轴的包装函数 */
static void tpSetRotaryUnlock(int axis, int unlock) {
    _SetRotaryUnlock(axis, unlock);
}

/** tpGetRotaryIsUnlocked — 查询旋转轴是否解锁的包装函数 */
static int tpGetRotaryIsUnlocked(int axis) {
    return _GetRotaryIsUnlocked(axis);
}

/*
 * tpCompleteSegment — 段完成后的清理
 * 当前段执行完毕且不在等待主轴信号时，从队列中弹出并执行清理操作
 */
static int tpCompleteSegment(TP_STRUCT * const tp,
        TC_STRUCT * const tc) {

    /* 如果在等待主轴到位信号，不移除 */
    if (tp->spindle.waiting_for_atspeed == tc->id) {
        return TP_ERR_FAIL;
    }
    if(tc->synchronized != TC_SYNC_NONE) {
        tp->spindle.offset += tc->target / tc->uu_per_rev;
    } else {
        tp->spindle.offset = 0.0;
    }

    /* 处理索引旋转轴 */
    if(tc->indexer_jnum != -1) {
        tpSetRotaryUnlock(tc->indexer_jnum, 0);
        if(tpGetRotaryIsUnlocked(tc->indexer_jnum))
            return TP_ERR_FAIL;
    }

    tc->active = 0;
    tc->remove = 0;
    tc->is_blending = 0;
    tc->splitting = 0;
    tc->cycle_time = tp->cycleTime;
    tc->currentvel = 0.0;
    tc->term_vel = 0.0;
    if (tp->reverse_run) {
        tcqBackStep(&tp->queue);
    } else {
        int res_pop = tcqPop(&tp->queue);
        if (res_pop) rtapi_print_msg(RTAPI_MSG_ERR,"Got error %d from tcqPop!\n", res_pop);
    }

    return TP_ERR_OK;
}


/*
 * tpHandleAbort — 处理中止命令
 * 根据当前运动状态处理中止命令的后果
 */
static tp_err_t tpHandleAbort(TP_STRUCT * const tp, TC_STRUCT * const tc,
        TC_STRUCT * const nexttc) {

    if(!tp->aborting) {
        return TP_ERR_NO_ACTION;
    }
    if( tc->currentvel == 0.0 && (!nexttc || nexttc->currentvel == 0.0))
    {
        tcqInit(&tp->queue);
        tp->goalPos = tp->currentPos;
        tp->done = 1;
        tp->depth = tp->activeDepth = 0;
        tp->aborting = 0;
        tp->execId = 0;
        tp->motionType = 0;
        tp->synchronized = 0;
        tp->reverse_run = 0;
        tpResume(tp);
        return TP_ERR_STOPPED;
    }
    return TP_ERR_SLOWING;
}


/*
 * tpActivateSegment — 激活轨迹段
 */
static tp_err_t tpActivateSegment(TP_STRUCT * const tp, TC_STRUCT * const tc) {

    if (!tc || tc->active) {
        return TP_ERR_OK;
    }

    if (!tp) {
        return TP_ERR_MISSING_INPUT;
    }

    if (tp->reverse_run && tc->synchronized != TC_SYNC_NONE) {
        return TP_ERR_REVERSE_EMPTY;
    }

    ///计算时间与长度特征 → 决定加速度模式
    ///arcBlendRampFreq：来自配置文件 INI 的"截止频率"（单位 Hz）
    ///TP_TIME_EPSILON 防止除零
    double cutoff_time = 1.0 / (fmax(emcmotConfig->arcBlendRampFreq, TP_TIME_EPSILON));

    ///计算此段剩余的位移长度
    double length = tcGetDistanceToGo(tc, tp->reverse_run);
    ///估算该段在当前速度条件下的执行时间
    double segment_time = 2.0 * length / (tc->currentvel + fmin(tc->finalvel,tpGetRealTargetVel(tp,tc)));

    ///RAMP 模式（简单线性加速度）
    if (segment_time < cutoff_time &&
            tc->canon_motion_type != 1 &&
            tc->term_cond == TC_TERM_COND_TANGENT &&
            tc->motion_type != TC_RIGIDTAP &&
            length != 0)
    {
        tc->accel_mode = TC_ACCEL_RAMP;
    }

    tc->active = 1;
    tp->motionType = tc->canon_motion_type;
    tc->blending_next = 0;
    tc->on_final_decel = 0;

    return TP_ERR_OK;
}


/*
 * tpSyncVelocityMode — 速度模式同步
 * 更新请求速度以跟随主轴速度（按进给速率缩放）
 */
static void tpSyncVelocityMode(TP_STRUCT * const tp, TC_STRUCT * const tc, TC_STRUCT * const nexttc) {

    if (nexttc && nexttc->synchronized) {
        nexttc->target_vel = tc->target_vel;
    }
}


/*
 * tpSyncPositionMode — 位置模式同步
 * 更新请求速度以跟踪主轴位置
 */
static void tpSyncPositionMode(TP_STRUCT * const tp, TC_STRUCT * const tc,
        TC_STRUCT * const nexttc ) {

    double spindle_vel, target_vel;
    double oldrevs = tp->spindle.revs;

    double pos_desired = (tp->spindle.revs - tp->spindle.offset) * tc->uu_per_rev;
    double pos_error = pos_desired - tc->progress;

    if(nexttc) {
        pos_error -= nexttc->progress;
    }

    if(tc->sync_accel) {
        double dt = fmax(tp->cycleTime, TP_TIME_EPSILON);
        spindle_vel = tp->spindle.revs / ( dt * tc->sync_accel++);
        target_vel = spindle_vel * tc->uu_per_rev;
        if(tc->currentvel >= target_vel) {

            tp->spindle.offset = tp->spindle.revs - tc->progress / tc->uu_per_rev;
            tc->sync_accel = 0;
            tc->target_vel = target_vel;
        } else {
            tc->target_vel = tc->maxvel;
        }
    } else {
        double errorvel;
        spindle_vel = (tp->spindle.revs - oldrevs) / tp->cycleTime;
        target_vel = spindle_vel * tc->uu_per_rev;
        errorvel = pmSqrt(fabs(pos_error) * tcGetTangentialMaxAccel(tc));
        if(pos_error<0) {
            errorvel *= -1.0;
        }
        tc->target_vel = target_vel + errorvel;
    }

    if (tc->target_vel < 0.0) {
        tc->target_vel = 0.0;
    }

    if (nexttc && nexttc->synchronized) {
        nexttc->target_vel = tc->target_vel;
    }
}


/*
 * tpDoParabolicBlending — 执行抛物线混合
 * 在段之间执行抛物线混合并处理状态更新
 */
static int tpDoParabolicBlending(TP_STRUCT * const tp, TC_STRUCT * const tc,
        TC_STRUCT * const nexttc) {


    tpUpdateBlend(tp,tc,nexttc);

    if(tc->currentvel > nexttc->currentvel) {
        tpUpdateMovementStatus(tp, tc);
    } else {
        tpUpdateMovementStatus(tp, nexttc);
    }

    emcmotStatus->current_vel = tc->currentvel + nexttc->currentvel;

    return TP_ERR_OK;
}


/*
 * tpUpdateCycle — 单段完整更新
 *负责计算运动的加速度、速度和位移，并检查运动段的结束条件
 *tp：指向轨迹规划器结构的指针
 *tc：指向当前轨迹组件的指针
 *nexttc：指向下一个轨迹分量的指针
 *
 * 【执行步骤】
 *   1. 保存更新前的位置（before）
 *   2. 获取当前位置
 *   3. 如果不混合，记录混合起始速度
 *   4. 计算加速度（ramp 或 trapezoidal）
 *   5. 根据加速度更新位移（tcUpdateDistFromAccel）
 *   6. 调试输出
 *   7. 检查段结束条件
 *   8. 计算位移（当前位置 - before）
 *   9. 更新规划器位置（tpAddCurrentPos）
 */
static int tpUpdateCycle(TP_STRUCT * const tp, TC_STRUCT * const tc, TC_STRUCT const * const nexttc)
{
    //保存执行任何更新之前的当前位置
    EmcPose before;

    tcGetPos(tc, &before);

    //如果不进行混合，则更新起始速度
    if (!tc->blending_next) {
        tc->vel_at_blend_start = tc->currentvel;
    }

    int res_accel = 1;//保存加速度计算结果的变量
    double acc=0, vel_desired=0;//当前周期的期望速度

    if (tc->accel_mode && tc->term_cond == TC_TERM_COND_TANGENT) {
        //计算加速度和期望速度
        res_accel = tpCalculateRampAccel(tp, tc, nexttc, &acc, &vel_desired);

    }

    //如果tpCalculateRampAccel失败, 代码回退到梯形加速度计算
    if (res_accel != TP_ERR_OK) {
        tpCalculateTrapezoidalAccel(tp, tc, nexttc, &acc, &vel_desired);
    }

    //根据加速度和期望速度更新行驶距离
    tcUpdateDistFromAccel(tc, acc, vel_desired, tp->reverse_run);
    tpDebugCycleInfo(tp, tc, nexttc, acc);

    //检查该段是否已到达终点
    tpCheckEndCondition(tp, tc, nexttc);

    //存储当前状态和先前状态之间位置差异的变量
    EmcPose displacement;

    tcGetPos(tc, &displacement);
    //计算当前位置与先前存储的位置之间的差异
    emcPoseSelfSub(&displacement, &before);

    rtapi_mutex_get(&emcmotInternal->command_mutex);
    int res_set = tpAddCurrentPos(tp, &displacement);
    rtapi_mutex_give(&emcmotInternal->command_mutex);

    return res_set;
}


/*
 * tpUpdateInitialStatus — 发送默认状态值
 * 重置状态结构中的默认值
 */
static int tpUpdateInitialStatus(TP_STRUCT const * const tp) {
    emcmotStatus->tcqlen = tcqLen(&tp->queue);
    emcmotStatus->requested_vel = 0.0;
    emcmotStatus->current_vel = 0.0;
    return TP_ERR_OK;
}


/*
 * tcSetSplitCycle — 标记需要分裂周期
 * 将段标记为"分裂"状态，并存储下一个周期的数据
 */
static inline int tcSetSplitCycle(TC_STRUCT * const tc, double split_time, double v_f)
{
    if (tc->splitting != 0 && split_time > 0.0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,"already splitting on id %d with cycle time %.16g, dx = %.16g, split time %.12g\n",
                tc->id,
                tc->cycle_time,
                tc->target-tc->progress,
                split_time);
        return TP_ERR_FAIL;
    }
    tc->splitting = 1;
    tc->cycle_time = split_time;
    tc->term_vel = v_f;
    return 0;
}


/*
 * tpCheckEndCondition — 检查段结束条件
 * 【核心逻辑】
 *   1. 计算剩余距离 dx 和平均速度 v_avg
 *   2. 估算完成段所需时间 dt
 *   3. 计算所需加速度 a_f
 *   4. 如果加速度超过限制，重新计算 dt 和 v_f
 *   5. 根据 dt 判断：正常 / 分裂周期 / 结束
 */
static int tpCheckEndCondition(TP_STRUCT const * const tp, TC_STRUCT * const tc, TC_STRUCT const * const nexttc) {

    tc->cycle_time = tp->cycleTime;
    double dx = tcGetDistanceToGo(tc, tp->reverse_run);

    if (dx <= TP_POS_EPSILON)
    {
        tc->progress = tcGetTarget(tc, tp->reverse_run);

        if (!tp->reverse_run) {
            tcSetSplitCycle(tc, 0.0, tc->currentvel);
        }
        if (tc->term_cond == TC_TERM_COND_STOP || tc->term_cond == TC_TERM_COND_EXACT || tp->reverse_run) {
            tc->remove = 1;
        }
        return TP_ERR_OK;
    } else if (tp->reverse_run) {
        return TP_ERR_NO_ACTION;
    } else if (tc->term_cond == TC_TERM_COND_STOP || tc->term_cond == TC_TERM_COND_EXACT) {
        return TP_ERR_NO_ACTION;
    }


    double v_f = tpGetRealFinalVel(tp, tc, nexttc);
    double v_avg = (tc->currentvel + v_f) / 2.0;

    double dt = TP_TIME_EPSILON / 2.0;
    if (v_avg > TP_VEL_EPSILON)
    {
        dt = fmax(dt, dx / v_avg);
    } else {
        if ( dx > (v_avg * tp->cycleTime) && dx > TP_POS_EPSILON) {
            return TP_ERR_NO_ACTION;
        }
    }

    double dv = v_f - tc->currentvel;
    double a_f = dv / dt;

    double a_max = tcGetTangentialMaxAccel(tc);

    double a = a_f;
    int recalc = sat_inplace(&a, a_max);

    if (recalc) {
        double disc = pmSq(tc->currentvel / a) + 2.0 / a * dx;
        if (disc < 0) {
            return TP_ERR_NO_ACTION;
        }

        if (disc < TP_TIME_EPSILON * TP_TIME_EPSILON) {
            dt =  -tc->currentvel / a;
        } else if (a > 0) {
            dt = -tc->currentvel / a + pmSqrt(disc);
        } else {
            dt = -tc->currentvel / a - pmSqrt(disc);
        }

        v_f = tc->currentvel + dt * a;
    }

    if (dt < TP_TIME_EPSILON) {
        tc->progress = tcGetTarget(tc, tp->reverse_run);
        tcSetSplitCycle(tc, 0.0, v_f);
    } else if (dt < tp->cycleTime ) {
        tcSetSplitCycle(tc, dt, v_f);
    } else
    {
        return TP_ERR_NO_ACTION;
    }

    rtapi_print_msg(RTAPI_MSG_DBG, " tc->splitting %d\n", tc->splitting);

    return TP_ERR_OK;
}


/*
 * tpHandleSplitCycle — 处理分裂周期
 *   1. 如果已标记 remove，不处理
 *   2. 将 progress 设为目标
 *   3. 计算位移并更新 tp 位置
 *   4. 标记 remove
 *   5. 将剩余时间分配给下一段
 *   6. 运行下一段的周期更新
 */
static int tpHandleSplitCycle(TP_STRUCT * const tp, TC_STRUCT * const tc,
        TC_STRUCT * const nexttc)
{
    if (tc->remove) {
        return TP_ERR_NO_ACTION;
    }

    EmcPose before;
    tcGetPos(tc, &before);

    tc->progress = tcGetTarget(tc,tp->reverse_run);
    EmcPose displacement;
    tcGetPos(tc, &displacement);
    emcPoseSelfSub(&displacement, &before);

    tpAddCurrentPos(tp, &displacement);

    tc->remove = 1;

    if (!nexttc) {
        return TP_ERR_OK;
    }

    switch (tc->term_cond) {
        case TC_TERM_COND_TANGENT:
            nexttc->cycle_time = tp->cycleTime - tc->cycle_time;
            nexttc->currentvel = tc->term_vel;
            break;
        case TC_TERM_COND_PARABOLIC:
            break;
        case TC_TERM_COND_STOP:
            break;
        case TC_TERM_COND_EXACT:
            break;
        default:
            rtapi_print_msg(RTAPI_MSG_ERR,"unknown term cond %d in segment %d\n",tc->term_cond,tc->id);
    }

    int queue_dir_step = tp->reverse_run ? -1 : 1;
    TC_STRUCT *next2tc = tcqItem(&tp->queue, queue_dir_step*2);

    tpUpdateCycle(tp, nexttc, next2tc);

    if (tc->cycle_time > nexttc->cycle_time && tc->term_cond == TC_TERM_COND_TANGENT) {
        tpUpdateMovementStatus(tp, tc);
    } else {
        tpUpdateMovementStatus(tp, nexttc);
    }

    return TP_ERR_OK;
}

/*
 * tpHandleRegularCycle — 处理常规周期
 * 处理不需要分裂的正常周期更新
 */
static int tpHandleRegularCycle(TP_STRUCT * const tp,
        TC_STRUCT * const tc,
        TC_STRUCT * const nexttc)
{
    if (tc->remove)
    {
        return TP_ERR_NO_ACTION;
    }
    tc->cycle_time = tp->cycleTime;

    tpUpdateCycle(tp, tc, nexttc);
    double v_this = 0.0, v_next = 0.0;

    double target_vel_this = tpGetRealTargetVel(tp, tc);
    double target_vel_next = tpGetRealTargetVel(tp, nexttc);

    tpComputeBlendVelocity(tc, nexttc, target_vel_this, target_vel_next, &v_this, &v_next, NULL);
    tc->blend_vel = v_this;
    if (nexttc) {
        nexttc->blend_vel = v_next;
    }

    if (nexttc && tcIsBlending(tc)) {
        tpDoParabolicBlending(tp, tc, nexttc);
    } else {
        tpUpdateMovementStatus(tp, tc);
    }
    return TP_ERR_OK;
}

/*
 * tpUpdateRigidTapState — 更新刚性攻丝状态
 * 处理刚性攻丝（钻孔-反转-退刀）的状态机转换
 */
static void tpUpdateRigidTapState(TP_STRUCT const * const tp, TC_STRUCT * const tc) {

    static double old_spindlepos;

    switch (tc->coords.rigidtap.state) {
        case RIGIDTAP_START:
            tc->coords.rigidtap.state = TAPPING;
        case TAPPING:

            if (tc->progress >= tc->coords.rigidtap.reversal_target) {
                tc->coords.rigidtap.state = REVERSING;
            }
            break;
        case REVERSING:

            // if (new_spindlepos < old_spindlepos) {
            //     PmCartesian start, end;
            //     PmCartLine *aux = &tc->coords.rigidtap.aux_xyz;
            //     // we've stopped, so set a new target at the original position
            //     tc->coords.rigidtap.spindlerevs_at_reversal = new_spindlepos + tp->spindle.offset;
            //
            //     pmCartLinePoint(&tc->coords.rigidtap.xyz, tc->progress, &start);
            //     end = tc->coords.rigidtap.xyz.start;
            //     pmCartLineInit(aux, &start, &end);
            //
            //     tc->coords.rigidtap.reversal_target = aux->tmag;
            //     tc->target = aux->tmag + 10. * tc->uu_per_rev;
            //     tc->progress = 0.0;
            //
            //
            //     tc->coords.rigidtap.state = RETRACTION;
            // }
            // old_spindlepos = new_spindlepos;
            break;
        case RETRACTION:
            // tc_debug_print("RETRACTION\n");
            // if (tc->progress >= tc->coords.rigidtap.reversal_target) {
            // 	emcmotStatus->spindle_status[tp->spindle.spindle_num].speed *= -1 / tc->coords.rigidtap.reversal_scale;
            //     tc->coords.rigidtap.state = FINAL_REVERSAL;
            // }
            break;
        case FINAL_REVERSAL:

            // if (new_spindlepos > old_spindlepos) {
            //     PmCartesian start, end;
            //     PmCartLine *aux = &tc->coords.rigidtap.aux_xyz;
            //     pmCartLinePoint(aux, tc->progress, &start);
            //     end = tc->coords.rigidtap.xyz.start;
            //     pmCartLineInit(aux, &start, &end);
            //     tc->target = aux->tmag;
            //     tc->progress = 0.0;
            //     //No longer need spindle sync at this point
            //     tc->synchronized = 0;
            //     tc->target_vel = tc->maxvel;
            //
            //     tc->coords.rigidtap.state = FINAL_PLACEMENT;
            // }
            // old_spindlepos = new_spindlepos;
            break;
        case FINAL_PLACEMENT:
            break;
    }
}


/*
 * tpRunCycle — 轨迹规划器主循环
 * 计算下一个时间步的目标位置
 */
int tpRunCycle(TP_STRUCT * const tp, long period)
{
    (void)period;
    TC_STRUCT *tc;
    TC_STRUCT *nexttc;

    /* 根据方向确定下一步的索引（正向+1，反向-1） */
    int queue_dir_step = tp->reverse_run ? -1 : 1;
    tc = tcqItem(&tp->queue, 0);
    nexttc = tcqItem(&tp->queue, queue_dir_step * 1);

    tpUpdateInitialStatus(tp);

    if(!tc) {
        tpHandleEmptyQueue(tp);
        return TP_ERR_WAITING;
    }
    if (tpHandleAbort(tp, tc, nexttc) == TP_ERR_STOPPED) {
        return TP_ERR_STOPPED;
    }

    int res_activate = tpActivateSegment(tp, tc);
    if (res_activate != TP_ERR_OK ) {
        return res_activate;
    }

    if (tc->motion_type == TC_RIGIDTAP) {
        tpUpdateRigidTapState(tp, tc);
    }

    switch (tc->synchronized) {
        case TC_SYNC_NONE:
            // emcmotStatus->spindleSync = 0;
            break;
        case TC_SYNC_VELOCITY:

            tpSyncVelocityMode(tp, tc, nexttc);
            break;
        case TC_SYNC_POSITION:

            tpSyncPositionMode(tp, tc, nexttc);
            break;
        default:

            break;
    }

#ifdef TC_DEBUG
    EmcPose pos_before = tp->currentPos;
#endif

    tcClearFlags(tc);
    tcClearFlags(nexttc);
    if (tc->splitting) {
        tpHandleSplitCycle(tp, tc, nexttc);
    } else {
        tpHandleRegularCycle(tp, tc, nexttc);
    }

#ifdef TC_DEBUG
    double mag;
    EmcPose disp;
    emcPoseSub(&tp->currentPos, &pos_before, &disp);
    emcPoseMagnitude(&disp, &mag);
    tc_debug_print("time: %.12e total movement = %.12e vel = %.12e\n",
            time_elapsed,
            mag, emcmotStatus->current_vel);

    tc_debug_print("tp_displacement = %.12e %.12e %.12e time = %.12e\n",
            disp.tran.x,
            disp.tran.y,
            disp.tran.z,
            time_elapsed);
#endif

    if (tc->remove) {
        tpCompleteSegment(tp, tc);
    }

    return TP_ERR_OK;
}

/*
 * tpSetSpindleSync — 设置主轴同步
 * 设置主轴同步模式和参数
 */
int tpSetSpindleSync(TP_STRUCT * const tp, int spindle, double sync, int mode)
{
    if(sync) {
        if (mode) {
            tp->synchronized = TC_SYNC_VELOCITY;
        } else {
            tp->synchronized = TC_SYNC_POSITION;
        }
        tp->uu_per_rev = sync;
        tp->spindle.spindle_num = spindle;
    } else
        tp->synchronized = 0;

    return TP_ERR_OK;
}

/*
 * tpPause — 暂停轨迹规划器
 */
int tpPause(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }
    tp->pausing = 1;
    return TP_ERR_OK;
}

/*
 * tpResume — 恢复轨迹规划器
 */
int tpResume(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }
    tp->pausing = 0;
    return TP_ERR_OK;
}

/*
 * tpAbort — 中止轨迹规划器
 */
int tpAbort(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    if (!tp->aborting) {
        tpPause(tp);
        tp->aborting = 1;
    }
    return TP_ERR_OK;
}

int tpGetMotionType(TP_STRUCT * const tp)
{
    return tp->motionType;
}

/*
 * tpGetPos — 获取规划器当前位置
 */
int tpGetPos(TP_STRUCT const * const tp, EmcPose * const pos)
{

    if (0 == tp) {
        ZERO_EMC_POSE((*pos));
        return TP_ERR_FAIL;
    } else {
        *pos = tp->currentPos;
    }

    return TP_ERR_OK;
}

/*
 * tpIsDone — 检查规划器是否完成
 */
int tpIsDone(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_OK;
    }

    return tp->done;
}

/*
 * tpQueueDepth — 获取队列深度
 */
int tpQueueDepth(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_OK;
    }

    return tp->depth;
}

/*
 * tpActiveDepth — 获取活跃深度
 */
int tpActiveDepth(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_OK;
    }

    return tp->activeDepth;
}

/*
 * tpSetRunDir — 设置运行方向
 */
int tpSetRunDir(TP_STRUCT * const tp, tc_direction_t dir)
{
    if (tpIsMoving(tp)) {
        return TP_ERR_FAIL;
    }

    switch (dir) {
        case TC_DIR_FORWARD:
        case TC_DIR_REVERSE:
            tp->reverse_run = dir;
        return TP_ERR_OK;
        default:
            rtapi_print_msg(RTAPI_MSG_ERR,"Invalid direction flag in SetRunDir");
        return TP_ERR_FAIL;
    }
}

/*
 * tpIsMoving — 检查规划器是否正在运动
 */
int tpIsMoving(TP_STRUCT const * const tp)
{
    if (emcmotStatus->current_vel >= TP_VEL_EPSILON )
    {
        return true;
    } else if (tp->spindle.waiting_for_index != MOTION_INVALID_ID || tp->spindle.waiting_for_atspeed != MOTION_INVALID_ID) {
        return true;
    }
    return false;
}
