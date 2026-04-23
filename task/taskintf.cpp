//
// Created by Administrator on 2025/8/16.
//
#include <cfloat>
#include <cmath>
#include <libnml/emccfg.h>
#include "hal/hal.h"
#include "libnml/emcglb.h"
#include "motion/emcmotcfg.h"
#include "motion/usrmotintf.h"
#include "motion/motion.h"

static struct TrajConfig_t TrajConfig;
static emcmot_command_t emcmotCommand;
static emcmot_status_t emcmotStatus;
static int new_config = 0;
int get_emcmot_internal_info = 0;
static emcmot_config_t emcmotConfig;
static emcmot_internal_t emcmotInternal;
static int localMotionCommandType = 0;
static int localMotionEchoSerialNumber = 0;

int emcTrajInit()
{
    int retval = 0;

    TrajConfig.Inited = 0;
    TrajConfig.Axis = 0;
    TrajConfig.MaxAccel = DBL_MAX;
    TrajConfig.AxisMask = 0;
    TrajConfig.LinearUnits = 1.0;
    TrajConfig.AngularUnits = 1.0;
    TrajConfig.MotionId = 0;
    TrajConfig.MaxVel = DEFAULT_TRAJ_MAX_VELOCITY;

	return retval;
}

int emcTrajDisable()
{
    emcmotCommand.command = EMCMOT_MOTION_DISABLE;

    return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcMotionUpdate()
{
    // read the emcmot status
    if (0 != usrmotReadEmcmotStatus(&emcmotStatus))
    {
	return -1;
    }
    new_config = 0;
    if (emcmotStatus.config_num != emcmotConfig.config_num) {
	if (0 != usrmotReadEmcmotConfig(&emcmotConfig)) {
	    return -1;
	}
	new_config = 1;
    }

    if (get_emcmot_internal_info) {
	if (0 != usrmotReadEmcmotInternal(&emcmotInternal)) {
	    return -1;
	}
    }

    // save the heartbeat and command number locally,
    // for use with emcMotionUpdate
    localMotionCommandType = emcmotStatus.commandEcho;
    localMotionEchoSerialNumber = emcmotStatus.commandNumEcho;

    return 0;
}

int emcJogAbort(int axis)
{
	if (axis < 0 || axis >= EMCMOT_MAX_AXIS) {
		return 0;
	}
	emcmotCommand.command = EMCMOT_JOG_ABORT;
	emcmotCommand.axis  = -1;

	return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcTrajAbort()
{
	emcmotCommand.command = EMCMOT_ABORT;

	return usrmotWriteEmcmotCommand(&emcmotCommand);
}

int emcMotionAbort()
{
	int r1;
	int r2;
	int r3 = 0;
	int t;

	r1 = -1;
	for (t = 0; t < EMCMOT_MAX_AXIS; t++) {
		if (0 == emcJogAbort(t)) {
			r1 = 0;		// at least one is okay
		}
	}

	r2 = emcTrajAbort();

	return (r1 == 0 && r2 == 0 && r3 == 0) ? 0 : -1;
}