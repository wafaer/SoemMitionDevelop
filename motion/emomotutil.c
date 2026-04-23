//
// Created by skeqi on 25-8-30.
//

#include "motion.h"

int emcmotErrorInit(emcmot_error_t * errlog)
{
    if (errlog == 0) {
        return -1;
    }

    errlog->head = 0;
    errlog->start = 0;
    errlog->end = 0;
    errlog->num = 0;
    errlog->tail = 0;

    return 0;
}