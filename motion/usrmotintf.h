//
// Created by Administrator on 2025/8/16.
//

#ifndef USRMOTINTF_H
#define USRMOTINTF_H

struct emcmot_status_t;
struct emcmot_command_t;
struct emcmot_config_t;
struct emcmot_internal_t;
struct emcmot_error_t;

#ifdef __cplusplus
extern "C" {
#endif

/* usrmotInit() initializes communication with the emcmot process */
    extern int usrmotInit(const char *name);
    extern int usrmotExit(void);
    extern int usrmotReadEmcmotStatus(emcmot_status_t * s);
    extern int usrmotWriteEmcmotCommand(emcmot_command_t * c);

    extern int usrmotReadEmcmotConfig(emcmot_config_t * s);
    extern int usrmotReadEmcmotInternal(emcmot_internal_t * s);


#define EMCMOT_COMM_OK 0	/* went through and honored */
#define EMCMOT_COMM_ERROR_CONNECT -1	/* can't even connect */
#define EMCMOT_COMM_ERROR_TIMEOUT -2	/* connected, but send timeout */
#define EMCMOT_COMM_ERROR_COMMAND -3	/* sent, but can't run command now */
#define EMCMOT_COMM_SPLIT_READ_TIMEOUT -4	/* can't read without split */
#define EMCMOT_COMM_INVALID_MOTION_ID -5 /* do not queue a motion id MOTION_INVALID_ID */

#ifdef __cplusplus
}
#endif

#endif //USRMOTINTF_H
