//
// Created by Administrator on 2025/8/19.
//

#ifndef SERVOENABLE_H
#define SERVOENABLE_H

#include "rtapi/rtapi_mutex.h"
#include "soem140/include/ethercat.h"
#include "servo_priv.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FREE = 0,                   /*空闲*/
    Absolute,
    Relative,
    Singleaxis,                 /*单轴*/
    Line,                       /*线性插补*/
    Cycle                       /*圆弧插补*/
} MOTION_TYPE_t;

typedef enum {
    S0X000 = 0,          /*上电→初始化*/
    S0X0250,            /*初始化→伺服无故障  故障→伺服无故障*/
    S0X0270,            /*初始化→伺服无故障*/
    S0X0231,            /*伺服无故障→伺服准备好   等待打开伺服使能→伺服准备好  伺服运行→伺服准备好 */
    S0X0233,            /*伺服准备好→等待打开伺服使能*/
    S0X0237,            /*等待打开伺服使能→伺服运行  快速停机→伺服运行*/
    S0X0217,            /*伺服运行→快速停机*/
    S0X021F,            /*故障停机*/
    S0X0218            /*故障停机→故障*/
} AXIS_STATUS_t;

typedef struct ecat_status_t {
    Motor_Out** motoro;
    Motor_In**  motori;
    int enable;
    MOTION_TYPE_t motype;
    AXIS_STATUS_t axstatus;
}ecat_status_t;

typedef struct ecat_config_t
{
    double ecatCycleTime;
}ecat_config_t;

typedef struct ecat_struct_t {
    rtapi_mutex_t command_mutex;
    struct ecat_status_t status;
    struct ecat_config_t config;
}ecat_struct_t;

    void *ecatthread(void *arg);
    void *ecatcheck(void *arg); 
    int ecatbringup(char *ifname);
    void *Motor_Enable(void *arg);
    void *Motor_Offable(void *arg);

int usrecatInit(const char *modname);
int ecat_thread_main(void);
int ecat_thread_exit(void);

#ifdef __cplusplus
}
#endif

#endif //SERVOENABLE_H
