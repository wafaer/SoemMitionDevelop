//
// Created by Administrator on 2025/8/16.
//

#include "usrmotintf.h"
#include <cstdio>
#include <cstring>

#include "emcmotcfg.h"
#include "libnml/_timer.h"
#include "motion/motion_struct.h"

extern "C" {
#include "rtapi/rtapi.h"
#include "motion/motion.h"
}

extern emcmot_command_t *emcmotCommand;
extern emcmot_status_t *emcmotStatus;
extern emcmot_config_t *emcmotConfig;
extern emcmot_internal_t *emcmotInternal;
extern emcmot_error_t *emcmotError;
extern emcmot_struct_t *emcmotStruct;

int usrmotWriteEmcmotCommand(emcmot_command_t * c)
{
    emcmot_status_t s;
    static int commandNum = 0;
    double end;

    if (!MOTION_ID_VALID(c->id)) {
        rtapi_print_msg(RTAPI_MSG_ERR, "USRMOT: ERROR: invalid motion id: %d\n",c->id);
        return EMCMOT_COMM_INVALID_MOTION_ID;
    }

    // c->commandNum = ++commandNum;

    /* check for mapped mem still around */
    if (0 == emcmotCommand) {
        rtapi_print_msg(RTAPI_MSG_ERR,"USRMOT: ERROR: can't connect to shared memory\n");
        return EMCMOT_COMM_ERROR_CONNECT;
    }

    /* copy entire command structure to shared memory */
    rtapi_mutex_get(&emcmotStruct->command_mutex);
    *emcmotCommand = *c;
    rtapi_mutex_give(&emcmotStruct->command_mutex);

    /* poll for receipt of command */
    /* set timeout for comm failure, now + timeout */
    end = etime() + DEFAULT_EMCMOT_COMM_TIMEOUT;
    /* now check to see if it got it */
    while (etime() < end) {
        /* update status */
        if (( usrmotReadEmcmotStatus(&s) == 0 ) && ( s.commandNumEcho == commandNum )) {
            /* now check emcmot status flag */
            if (s.commandStatus == EMCMOT_COMMAND_OK) {
                return EMCMOT_COMM_OK;
            } else {
                rtapi_print_msg(RTAPI_MSG_ERR, "USRMOT: ERROR: invalid command\n");
                return EMCMOT_COMM_ERROR_COMMAND;
            }
        }
        esleep(25e-6);
    }
    rtapi_print_msg(RTAPI_MSG_ERR, "USRMOT: ERROR: command %u timeout (seq: %d)\n", c->command, commandNum);
    return EMCMOT_COMM_ERROR_TIMEOUT;
}

int usrmotReadEmcmotStatus(emcmot_status_t * s)
{
    int split_read_count;

    /* check for shmem still around */
    if (0 == emcmotStatus) {
        return EMCMOT_COMM_ERROR_CONNECT;
    }
    split_read_count = 0;
    do {
        if(split_read_count > 0) esleep(1e-6);	// Don't busy-loop and give time to process
        /* copy status struct from shmem to local memory */
        memcpy(s, emcmotStatus, sizeof(emcmot_status_t));
        /* got it, now check head-tail matche */
        if (s->head == s->tail) {
            /* head and tail match, done */
            return EMCMOT_COMM_OK;
        }
        /* inc counter and try again, max three times */
    } while ( ++split_read_count < 3 );
    /* A timeout is harmless. It will be tried again, soon enough */
    /* rcs_print("%s: Split read timeout\n", __FUNCTION__); */
    return EMCMOT_COMM_SPLIT_READ_TIMEOUT;
}

int usrmotReadEmcmotConfig(emcmot_config_t * s)
{
    int split_read_count;

    /* check for shmem still around */
    if (0 == emcmotConfig) {
	return EMCMOT_COMM_ERROR_CONNECT;
    }
    split_read_count = 0;
    do {
	if(split_read_count > 0) esleep(1e-6);	// Don't busy-loop and give time to process
	/* copy config struct from shmem to local memory */
	memcpy(s, emcmotConfig, sizeof(emcmot_config_t));
	/* got it, now check head-tail matches */
	if (s->head == s->tail) {
	    /* head and tail match, done */
	    return EMCMOT_COMM_OK;
	}
	/* inc counter and try again, max three times */
    } while ( ++split_read_count < 3 );
    return EMCMOT_COMM_SPLIT_READ_TIMEOUT;
}

/* copies internal to s */
int usrmotReadEmcmotInternal(emcmot_internal_t * s)
{
    int split_read_count;

    /* check for shmem still around */
    if (0 == emcmotInternal) {
	return EMCMOT_COMM_ERROR_CONNECT;
    }
    split_read_count = 0;
    do {
	if(split_read_count > 0) esleep(1e-6);	// Don't busy-loop and give time to process
	/* copy debug struct from shmem to local memory */
	memcpy(s, emcmotInternal, sizeof(emcmot_internal_t));
	/* got it, now check head-tail matches */
	if (s->head == s->tail) {
	    /* head and tail match, done */
	    return EMCMOT_COMM_OK;
	}
	/* inc counter and try again, max three times */
    } while ( ++split_read_count < 3 );
    return EMCMOT_COMM_SPLIT_READ_TIMEOUT;
}
