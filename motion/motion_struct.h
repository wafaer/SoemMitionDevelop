//
// Created by Administrator on 2025/8/16.
//

#ifndef MOTION_STRUCT_H
#define MOTION_STRUCT_H

#include <rtapi/rtapi_mutex.h>
#include <motion/motion.h>

/* big comm structure, for upper memory */
    typedef struct emcmot_struct_t {
        rtapi_mutex_t command_mutex;  // Used to protect access to `command`.
        struct emcmot_command_t command;   /* struct used to pass commands/data from Task to Motion */

        struct emcmot_status_t status;	/* Struct used to store RT status */
        struct emcmot_config_t config;	/* Struct used to store RT config */
        struct emcmot_error_t error;	/* ring buffer for error messages */
        struct emcmot_internal_t internal;	/* Struct used to store RT status and debug
                       data - 2nd largest block */
    } emcmot_struct_t;


#endif //MOTION_STRUCT_H
