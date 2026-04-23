//
// Created by Administrator on 2025/8/15.
//

#ifndef TP_H
#define TP_H
#include "tp_types.h"
#include "tc_types.h"
#include "motion/motion.h"
#include "tcq.h"

// functions not used by motmod:
int tpAddCurrentPos(TP_STRUCT * const tp, EmcPose const * const disp);
int tpSetCurrentPos(TP_STRUCT * const tp, EmcPose const * const pos);

int tpIsMoving(TP_STRUCT const * const tp);
int tpInit(TP_STRUCT * const tp);

// functions used by motmod:
int tpCreate(TP_STRUCT * const tp, int _queueSize,int id);
int tpClear(TP_STRUCT * const tp);
int tpSetCycleTime(TP_STRUCT * tp, double secs);
int tpSetVmax(TP_STRUCT * tp, double vmax, double ini_maxvel);
int tpSetVlimit(TP_STRUCT * tp, double limit);
int tpSetAmax(TP_STRUCT * tp, double amax);
int tpSetId(TP_STRUCT * tp, int id);
int tpGetExecId(TP_STRUCT * tp);
struct state_tag_t tpGetExecTag(TP_STRUCT * const tp);
int tpSetTermCond(TP_STRUCT * tp, int cond, double tolerance);
int tpSetPos(TP_STRUCT * tp, EmcPose const * const pos);
int tpRunCycle(TP_STRUCT * tp, long period);
int tpPause(TP_STRUCT * tp);
int tpResume(TP_STRUCT * tp);
int tpAbort(TP_STRUCT * tp);
int tpAddLine(TP_STRUCT * const tp, EmcPose end, int canon_motion_type,
			double vel, double ini_maxvel, double acc, unsigned char enables,
			char atspeed, int indexer_jnum);
int tpAddCircle(TP_STRUCT * const tp, EmcPose end, PmCartesian center,
		PmCartesian normal, int turn, int canon_motion_type, double vel,
		double ini_maxvel, double acc, unsigned char enables,
		char atspeed);
int tpGetPos(TP_STRUCT const  * const tp, EmcPose * const pos);
int tpIsDone(TP_STRUCT * const tp);
int tpQueueDepth(TP_STRUCT * const tp);
int tpActiveDepth(TP_STRUCT * const tp);
int tpGetMotionType(TP_STRUCT * const tp);
int tpSetSpindleSync(TP_STRUCT * const tp, int spindle, double sync, int wait);

int tpSetAout(TP_STRUCT * const tp, unsigned char index, double start, double end);
int tpSetDout(TP_STRUCT * const tp, int index, unsigned char start, unsigned char end); //gets called to place DIO toggles on the TC queue

int tpSetRunDir(TP_STRUCT * const tp, tc_direction_t dir);

void tpMotFunctions(void(  *pSetRotaryUnlock)(int,int) ,int (  *pGetRotaryIsUnlocked)(int), double(*paxis_get_vel_limit)(int),double(*paxis_get_acc_limit)(int));
void tpMotData(emcmot_status_t * ,emcmot_config_t *);

static double estimateParabolicBlendPerformance(
		TP_STRUCT const *tp,
		TC_STRUCT const *tc,
		TC_STRUCT const *nexttc);
static int tcRotaryMotionCheck(TC_STRUCT const * const tc);
static inline double getMaxFeedScale(TC_STRUCT const * tc);
static double tpGetTangentKinkRatio(void);
static int tpUpdateCycle(TP_STRUCT * const tp,
		TC_STRUCT * const tc, TC_STRUCT const * const nexttc) ;
static int tpCheckEndCondition(TP_STRUCT const * const tp, TC_STRUCT * const tc, TC_STRUCT const * const nexttc);
#endif //TP_H
