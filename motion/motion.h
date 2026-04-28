//
// Created by Administrator on 2025/8/14.
//

#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>
#include <limits.h>
#include <stdio.h>

#include "emcmotcfg.h"
#include "simple_tp.h"
#include "kinematics/cubic.h"
#include "tp/tp_types.h"

#define MOTION_INVALID_ID INT_MIN
#define MOTION_ID_VALID(x) ((x) != MOTION_INVALID_ID)

#ifdef __cplusplus
extern "C" {
#endif

#include "libnml/emc.h"
#include "rtapi/rtapi_mutex.h"

typedef enum {
		EMCMOT_FREE   =0,                 /* no cmd */

		EMCMOT_DOWNLOADS_CONFIG = 1,     /* set system config */

		EMCMOT_AXIS_ENABLE = 2,		/* enable servos for active joints */
		EMCMOT_AXIS_DISABLE = 3,		/* disable servos for active joints */
		EMCMOT_MOTION_ENABLE = 4,
		EMCMOT_MOTION_DISABLE = 5,

		EMCMOT_ABORT = 6,	/* abort all motion */
		EMCMOT_PAUSE = 7,		/* pause motion */
		EMCMOT_RESUME = 8,		/* resume motion */

		EMCMOT_JOG_CONT = 9,	/* continuous jog */
		EMCMOT_JOG_INCR = 10,	/* incremental jog */
		EMCMOT_JOG_ABS = 11,		/* absolute jog */
		EMCMOT_JOG_ABORT = 12,               /* abort one joint num or axis num */

		EMCMOT_SET_LINE = 13,	/* queue up a linear move */
		EMCMOT_SET_CIRCLE = 14,	/* queue up a circular move */

		EMCMOT_SET_FREE = 15,
		EMCMOT_SET_COORD = 16,

    } cmd_code_t;

	typedef struct emcmot_command_t
	{
		cmd_code_t command;	/* command code (enum) */
		int commandNum;		/* increment this for new command */
		double maxLimit;	/* pos value for position limit, output */
		double minLimit;	/* neg value for position limit, output */
		EmcPose pos;		/* line/circle endpt, or teleop vector */
		PmCartesian center;	/* center for circle */
		PmCartesian normal;	/* normal vec for circle */
		int turn;		/* turns for circle or joint number for a locking indexer*/
		double Maxvel;		/* max velocity */
		double Maxacc;		/* max acceleration */
		double Maxdec;		/* max acceleration */

		double vel;		/* velocity */
		double acc;		/* acceleration */
		double dec;		/* acceleration */

	    double ini_maxvel;      /* max velocity allowed by machine constraints (the INI file) */
	    int motion_type;        /* this move is because of traverse, feed, arc, or toolchange */
		double spindlesync;     /* user units per spindle revolution, 0 = no sync */
		int dir;
		int ref;
		int pulse;
		double backlash;	/* amount of backlash */
		int id;			/* id for motion */
		int termCond;		/* termination condition 终止条件*/
		double tolerance;	/* tolerance for path deviation in CONTINUOUS mode */
		int axis;		/* which axis index to use for below */
		int spindle; 	/* which spindle to use */
		double scale;		/* velocity scale or spindle_speed scale arg */
		double offset;		/* input, output, or home offset arg */

		int32_t minFerror;	/* min following error */
		int32_t maxFerror;	/* max following error */

		unsigned char start, end;	/* these are related to synched AOUT/DOUT. now=whether now or synched, out = which gets set, start=start value, end=end value */

		double comp_nominal, comp_forward, comp_reverse; /* compensation triplet, nominal, forward, reverse */

		double maxAxisScale;

	    int arcBlendOptDepth;
	    int arcBlendEnable;
	    int arcBlendFallbackEnable;
	    int arcBlendGapCycles;
	    double arcBlendRampFreq;
	    double arcBlendTangentKinkRatio;
    } emcmot_command_t;


/* this enum lists the possible results of a command */

    typedef enum {
	EMCMOT_COMMAND_OK = 0,	/* cmd honored */
	EMCMOT_COMMAND_UNKNOWN_COMMAND,	/* cmd not understood */
	EMCMOT_COMMAND_INVALID_COMMAND,	/* cmd can't be handled now */
	EMCMOT_COMMAND_INVALID_PARAMS,	/* bad cmd params */
	EMCMOT_COMMAND_BAD_EXEC	/* error trying to initiate */
    } cmd_status_t;

	typedef enum {
		EMC_MOTION_TYPE_TRAVERSE = 1,
		EMC_MOTION_TYPE_FEED = 2,
		EMC_MOTION_TYPE_ARC = 3
	} emcmot_type_t;

/* termination conditions for queued motions */
#define EMCMOT_TERM_COND_STOP 1
#define EMCMOT_TERM_COND_BLEND 2
#define EMCMOT_TERM_COND_TANGENT 3

    typedef unsigned short EMCMOT_MOTION_FLAG;


/* bit masks */
#define EMCMOT_MOTION_ENABLE_BIT      0x0001
#define EMCMOT_MOTION_INPOS_BIT       0x0002
#define EMCMOT_MOTION_FREE_BIT        0x0004
#define EMCMOT_MOTION_COORD_BIT       0x0008
#define EMCMOT_MOTION_ERROR_BIT       0x0010
#define EMCMOT_MOTION_TELEOP_BIT      0x0020

/* joint flag type */
    typedef unsigned short EMCMOT_JOINT_FLAG;


/* bit masks */
#define EMCMOT_AXIS_ENABLE_BIT         0x0001
#define EMCMOT_AXIS_ACTIVE_BIT         0x0002
#define EMCMOT_AXIS_INPOS_BIT          0x0004
#define EMCMOT_AXIS_ERROR_BIT          0x0008
#define EMCMOT_AXIS_MAX_HARD_LIMIT_BIT 0x0010
#define EMCMOT_AXIS_MIN_HARD_LIMIT_BIT 0x0020
#define EMCMOT_AXIS_FERROR_BIT         0x0040
#define EMCMOT_AXIS_FAULT_BIT          0x0080

/* motion controller states */

    typedef enum {
	EMCMOT_MOTION_DISABLED = 0,
	EMCMOT_MOTION_FREE,
	EMCMOT_MOTION_COORD
    } motion_state_t;

/* flags for enabling spindle scaling, feed scaling,
   adaptive feed, and feed hold */

#define SS_ENABLED 0x01
#define FS_ENABLED 0x02
#define AF_ENABLED 0x04
#define FH_ENABLED 0x08

    typedef struct
	{
		/* configuration info - changes rarely */
		int type;		/* 0 = linear, 1 = rotary */
		double max_pos_limit;	/* upper soft limit on joint pos */
		double min_pos_limit;	/* lower soft limit on joint pos */
		double max_jog_limit;	/* jog limits change when not homed */
		double min_jog_limit;
		double vel_limit;	/* upper limit of joint speed */
		double acc_limit;	/* upper limit of joint accel */
		double min_ferror;	/* zero speed following error limit */
		double max_ferror;	/* max speed following error limit */

		EMCMOT_JOINT_FLAG flag;	/* see above for bit details */
		double coarse_pos;	    /* trajectory point, before interp */
		double pos_cmd;		/* commanded joint position */
		double vel_cmd;		/* commanded joint velocity */
		double acc_cmd;		/* commanded joint acceleration */
		double motor_pos_cmd;	/* commanded position, with comp */
		double motor_pos_fb;	/* position feedback, with comp */
		double pos_fb;		/* position feedback, comp removed */
		double ferror;		/* following error */
		double ferror_limit;	/* limit depends on speed */
		double ferror_high_mark;	/* max following error */
		simple_tp_t free_tp;	/* planner for free mode motion */

		/* internal info - changes regularly, not usually accessed from user
		   space */
		CUBIC_STRUCT cubic;	/* cubic interpolator data */

		int on_pos_limit;	/* non-zero if on limit */
		int on_neg_limit;	/* non-zero if on limit */

		double motor_offset;	/* diff between internal and motor pos, used to set position to zero during homing */
    } emcmot_axis_t;

    typedef struct {
		EMCMOT_JOINT_FLAG flag;	/* see above for bit details */
	    double axis_vel_cmd;	/* commanded axis velocity */
		double pos_cmd;		/* commanded joint position */
		double pos_fb;		/* position feedback, comp removed */
		double vel_cmd;         /* current velocity */
		double acc_cmd;         /* current acceleration */
		double ferror;		/* following error */
		double ferror_high_mark;	/* max following error */
		double backlash;	/* amount of backlash */
		double max_pos_limit;	/* upper soft limit on joint pos */
		double min_pos_limit;	/* lower soft limit on joint pos */
		double min_ferror;	/* zero speed following error limit */
		double max_ferror;	/* max speed following error limit */
    } emcmot_axis_status_t;

    typedef struct emcmot_status_t {
		unsigned char head;	/* flag count for mutex detect */
		/* these three are updated only when a new command is handled */
		cmd_code_t commandEcho;	/* echo of input command */
		int commandNumEcho;	/* echo of input command number */
		cmd_status_t commandStatus;	/* result of most recent command */
		/* these are config info, updated when a command changes them */
		double feed_scale;	/* velocity scale factor for all motion but rapids */
		double rapid_scale;	/* velocity scale factor for rapids */
		unsigned char enables_new;	/* flags for FS, SS, etc */
			/* the above set is the enables in effect for new moves */
		/* the rest are updated every cycle */
		double net_feed_scale;	/* net scale factor for all motion */
		unsigned char enables_queued;	/* flags for FS, SS, etc */
			/* the above set is the enables in effect for the
			   currently executing move */
		motion_state_t motion_state; /* operating state: FREE, COORD, etc. */
		EMCMOT_MOTION_FLAG motionFlag;	/* see above for bit details */
		EmcPose carte_pos_cmd;	/* commanded Cartesian position */
		int carte_pos_cmd_ok;	/* non-zero if command is valid */
		EmcPose carte_pos_fb;	/* actual Cartesian position */
		int carte_pos_fb_ok;	/* non-zero if feedback is valid */
	    emcmot_axis_status_t axis_status[EMCMOT_MAX_AXIS];	/* all axis status data */

		int on_soft_limit;	/* non-zero if any joint is on soft limit */

		int misc_error[EMCMOT_MAX_MISC_ERROR]; /* Random Error pins*/

		/* dynamic status-- changes every cycle */
		unsigned int heartbeat;
		int config_num;		/* incremented whenever configuration
					   changed. */
		int id;			/* id for executing motion */
		int depth;		/* motion queue depth */
		int activeDepth;	/* depth of active blend elements */
		int queueFull;		/* Flag to indicate the tc queue is full */
		int paused;		/* Flag to signal motion paused */
		int overrideLimitMask;	/* non-zero means one or more limits ignored */
					/* 1 << (joint-num*2) = ignore neg limit */
					/* 2 << (joint-num*2) = ignore pos limit */
	    int reverse_run;

		/* static status-- only changes upon input commands, e.g., config */
		double Maxvel;		/* scalar max vel */
		double Maxacc;		/* scalar max accel */

		int motionType;
		double distance_to_go;  /* in this move */
		EmcPose dtg;
		double current_vel;
		double requested_vel;

		unsigned int tcqlen;
		unsigned char tail;	/* flag count for mutex detect */
		int external_offsets_applied;
		EmcPose eoffset_pose;

	    int stepping;
	    bool jogging_active;
    } emcmot_status_t;

    typedef struct emcmot_config_t
	{
		unsigned char head;	/* flag count for mutex detect */

		int config_num;		/* Incremented everytime configuration
					   changed, should match status.config_num */
		int numAxes;		/* The number of total joints in the system (which
					   must be between 1 and EMCMOT_MAX_JOINTS,
					   inclusive). includes extra joints*/

	    int numMiscError;     /* userdefined number of Misc Errors. default is 0.
	                                  but can be altered at motmod insmod time */


		double trajCycleTime;	/* the rate at which the trajectory loop
					   runs.... (maybe) */
		double servoCycleTime;	/* the rate of the servo loop - Not the same
					   as the traj time */

	    int interpolationRate;	/* grep control.c for an explanation....
					   approx line 50 */

		double limitVel;	/* scalar upper limit on vel */
		unsigned char tail;	/* flag count for mutex detect */
	    int arcBlendOptDepth;
	    int arcBlendEnable;
	    int arcBlendFallbackEnable;
	    int arcBlendGapCycles;
	    double arcBlendRampFreq;
	    double arcBlendTangentKinkRatio;

	    double maxAxisScale;
    } emcmot_config_t;

/* error structure - A ring buffer used to pass formatted printf strings to usr space */
    typedef struct emcmot_error_t {
		unsigned char head;	/* flag count for mutex detect */
		int start;		/* index of oldest error */
		int end;		/* index of newest error */
		int num;		/* number of items */
		unsigned char tail;	/* flag count for mutex detect */
    } emcmot_error_t;


typedef struct emcmot_internal_t {
		rtapi_mutex_t command_mutex;
	    unsigned char head; /* flag count for mutex detect */
	    unsigned char tail; /* flag count for mutex detect */
	    int split;          /* number of split command reads */
	    int enabling;       /* starts up disabled */
		int freeing;
	    int coordinating;   /* starts up in free mode */
	    int teleoperating;  /* starts up in free mode */
	    int overriding;     /* non-zero means we've initiated an joint
	                           move while overriding limits */
	    TP_STRUCT coord_tp; /* coordinated mode planner */
	    int idForStep;      /* status id while stepping */
    } emcmot_internal_t;

/* error ring buffer access functions */
    extern int emcmotErrorInit(emcmot_error_t * errlog);
    extern int emcmotErrorPut(emcmot_error_t * errlog, const char *error);
    extern int emcmotErrorPutfv(emcmot_error_t * errlog, const char *fmt, va_list ap);
    extern int emcmotErrorPutf(emcmot_error_t * errlog, const char *fmt, ...);
    extern int emcmotErrorGet(emcmot_error_t * errlog, char *error);



	//函数定义
	static int tp_init();
	int motion_thread_main(void);
	void updata_axis_param(int numAxes);
	void motion_thread_exit(void);
	static int init_motion_comm_buffers(void);
	void emcmot_config_change(void);
	static int init_motion_threads(void);
	void emcmotSetCycleTime(unsigned long nsec );
	static int setTrajCycleTime(double secs);
	static int setServoCycleTime(double secs);

#ifdef __cplusplus
}
#endif

#endif //MOTION_H
