//
// Created by Administrator on 2025/8/19.
//

#include "ServoEnable.h"
#include "hal/hal.h"
#include <time.h>
#include <sys/time.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#include <stdlib.h>

#include "motion/emcmotcfg.h"
#include "motion/motion_priv.h"
#include "rtapi/rtapi.h"
#include "soem140/include/osal.h"


#define EC_TIMEOUTMON 500    // 监控超时时间常量（毫秒）
#define NSEC_PER_SEC  1000000000 // 每秒的纳秒数（1e9）
#define USEC_PER_SEC  1000000 
#define RTAPI_CLOCK (CLOCK_MONOTONIC)

static uint8 IOmap[4096];    // IO map 缓冲（进/出站数据区域），大小示例 4096 字节
static OSAL_THREAD_HANDLE threadrt, thread1; // 线程句柄：一个实时循环线程，一个检查线程
static int expectedWKC;      // 期待的工作计数（Working Counter），用于验证帧的正确性
static int wkc;              // 实际收到的工作计数
static int mappingdone, dorun, inOP, dowkccheck; // 状态标志：映射完成、是否允许运行、是否在 OP、WKC 检查计数
static int currentgroup = 0; // 当前组索引（SOEM 支持分组）
static int cycle = 0;        // 循环次数计数器
static int64_t cycletime = 1000000; // 周期时间，默认 1,000,000 ns（即 1 ms）
static ecx_contextt ctx;    // SOEM 主上下文结构，包含网络和从站信息
static int enprintf = 0;
static int disenprintf = 0;

static int ecat_comp_id;
ecat_struct_t *emcecatStruct = 0;
ecat_status_t *emcecatStatus = 0;
ecat_config_t *emcecatConfig = 0;
extern emcmot_hal_data_t *emcmot_hal_data;
extern emcmot_status_t *emcmotStatus;
extern emcmot_axis_t axes[EMCMOT_MAX_AXIS];
extern int num_axis;
extern emcmot_internal_t *emcmotInternal;

atomic_int_fast32_t atomic_actpos[EMCMOT_MAX_AXIS];
atomic_int_fast32_t atomic_targetpos[EMCMOT_MAX_AXIS];

Motor_Out motorOutInstance[EMCMOT_MAX_AXIS] = {0};
Motor_In  motorInInstance[EMCMOT_MAX_AXIS]  = {0};

static int firstflag = 0;
static int32_t InPos;
static int32_t ActPos;
static int64_t times_c = 0;
static u_int16_t controlword = 0;

typedef struct{
   struct timespec start_time;
   struct timespec end_time;
}microtimer_t;

static int Motor_PDO_setup(uint16 slave);
static int init_ecat_comm_buffers(void);
static int init_ecat_threads(void);
static int setEcatCycleTime(double secs);

static uint16_t read_status_word(uint16_t slave_index);
static void verify_pdo_mapping(uint16_t slave_index);

void timer_start(microtimer_t *timer)
{
   clock_gettime(CLOCK_MONOTONIC, &timer->start_time);
}
void timer_end(microtimer_t *timer)
{
   clock_gettime(CLOCK_MONOTONIC, &timer->end_time);
}
long long timer_elapsed(microtimer_t *timer)
{
   long long sec_to_us = (timer->end_time.tv_sec - timer->start_time.tv_sec) * 1000000LL;
   long long ns_to_us = (timer->end_time.tv_nsec - timer->start_time.tv_nsec) / 1000LL;
   return sec_to_us + ns_to_us;
}

static float pgain = 0.01f;   // 比例项
static float igain = 0.00002f; // 积分项
static int64 syncoffset = 500; // 同步偏移示例：500,000 ns（即 500 us）
static int64 timeerror; // 上一次计算的时间误差

/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
   static int64 integral = 0; // 静态保存积分项（累加误差）
   int64 delta;
   delta = (reftime - syncoffset) % cycletime; // 以 cycletime 为周期计算差值
   if (delta > (cycletime / 2))
   {
      delta = delta - cycletime; // 将差值规范化到 [-cycletime/2, +cycletime/2]
   }
   timeerror = -delta; // 记录时间误差（取负）
   integral += timeerror; // 积分项累加
   *offsettime = (int64)((timeerror * pgain) + (integral * igain)); // PI 输出，作为下一周期的时间偏移调整
}

/* Cyclic RT EtherCAT thread */
void *ecatthread(void *arg)
{
   // // static int64_t toff = 0;
   // // static int cycle_count = 0;
   // // axis_hal_t *axis_data;
   // //
   // // if(firstflag == 0)
   // // {
   // //    emcecatStatus->motoro = calloc(num_axis, sizeof(Motor_Out*));
   // //    emcecatStatus->motori = calloc(num_axis, sizeof(Motor_In*));
   // //
   // //    for (int axis_num = 1; axis_num <= num_axis; axis_num++)
   // //    {
   // //       emcecatStatus->motoro[axis_num - 1] = (Motor_Out*)(ec_slave[axis_num].outputs);
   // //       emcecatStatus->motori[axis_num - 1] = (Motor_In*)(ec_slave[axis_num].inputs);
   // //       emcecatStatus->motoro[axis_num - 1]->workModeOut = 8;
   // //
   // //       InPos = emcecatStatus->motori[axis_num - 1]->actualPosition;
   // //       emcecatStatus->motoro[axis_num - 1]->PPtargetPosition = InPos;
   // //
   // //       atomic_store(&atomic_actpos[axis_num - 1], emcecatStatus->motori[axis_num - 1]->actualPosition);
   // //       atomic_store(&atomic_targetpos[axis_num - 1], emcecatStatus->motoro[axis_num - 1]->PPtargetPosition);
   // //
   // //       axes[axis_num].pos_cmd = emcecatStatus->motoro[axis_num - 1]->PPtargetPosition;
   // //       axes[axis_num].pos_fb = emcecatStatus->motori[axis_num - 1]->actualPosition;
   // //
   // //       rtapi_print_msg(RTAPI_MSG_INFO, "axis_num %d actualPosition=%d\n", axis_num, emcecatStatus->motori[axis_num - 1]->actualPosition);
   // //       rtapi_print_msg(RTAPI_MSG_INFO, "axis_num %d PPtargetPosition=%d\n", axis_num, emcecatStatus->motoro[axis_num - 1]->PPtargetPosition);
   // //    }
   // //
   // //    ec_send_processdata();
   // //
   // //    firstflag = 1;
   // // }
   // //
   // // int retval = ec_receive_processdata(EC_TIMEOUTRET);
   // //
   // // for (int axis_index = 0; axis_index < num_axis; axis_index++)
   // // {
   // //
   // //    rtapi_print_msg(RTAPI_MSG_INFO, "axis_num=%d actpos=%d\n", axis_index, emcecatStatus->motori[axis_index]->actualPosition);
   // //
   // //
   // //    axes[axis_index].free_tp.curr_pos = emcecatStatus->motori[axis_index]->actualPosition;
   // //    axis_data = &(emcmot_hal_data->axis[axis_index]);
   // //
   // //    *(axis_data->motor_pos_fb) = emcecatStatus->motori[axis_index]->actualPosition;
   // //
   // //    atomic_store(&atomic_actpos[axis_index], emcecatStatus->motori[axis_index]->actualPosition);
   // //
   // //    u_int16_t status = emcecatStatus->motori[axis_index]->statusword & 0x03FF;
   // //    if (*(emcmot_hal_data->axis[axis_index].amp_enable))
   // //    {
   // //       switch (status)
   // //       {
   // //       case 0x0000:
   // //          break;
   // //       case 0x0218:
   // //          break;
   // //       case 0x0250:
   // //          emcecatStatus->motoro[axis_index]->controlword = 0x0006;
   // //          break;
   // //       case 0x0270:
   // //          emcecatStatus->motoro[axis_index]->controlword = 0x0006;
   // //          break;
   // //       case 0x0231:
   // //          emcecatStatus->motoro[axis_index]->controlword = 0x0007;
   // //          break;
   // //       case 0x0233:
   // //          emcecatStatus->motoro[axis_index]->controlword = 0x000F;
   // //          break;
   // //       case 0x0237:
   // //          emcecatStatus->motoro[axis_index]->controlword = 0x000F;
   // //          break;
   // //       case 0x021F:
   // //          emcecatStatus->motoro[axis_index]->controlword = 0x80;
   // //          break;
   // //       }
   // //    }else
   // //    {
   // //       switch (status)
   // //       {
   // //       case 0x0000:
   // //          break;
   // //       case 0x0218:
   // //          break;
   // //       case 0x0250:
   // //          emcecatStatus->motoro[axis_index]->controlword = 0x0006;
   // //          break;
   // //       case 0x0270:
   // //          emcecatStatus->motoro[axis_index]->controlword = 0x0006;
   // //          break;
   // //       case 0x0231:
   // //          emcecatStatus->motoro[axis_index]->controlword = 0x0006;
   // //          break;
   // //       case 0x0233:
   // //          emcecatStatus->motoro[axis_index]->controlword = 0x0006;
   // //          break;
   // //       case 0x0237:
   // //          emcecatStatus->motoro[axis_index]->controlword = 0x0006;
   // //          break;
   // //       case 0x021F:
   // //          emcecatStatus->motoro[axis_index]->controlword = 0x80;
   // //          break;
   // //       }
   // //    }
   // //
   // //
   // //    emcecatStatus->motoro[axis_index]->PPtargetPosition = atomic_load(&atomic_targetpos[axis_index]);
   // //
   // //    rtapi_print_msg(RTAPI_MSG_INFO, "axis_num=%d targetpos=%d", axis_index, emcecatStatus->motoro[axis_index]->PPtargetPosition);
   // //
   // //    if(ec_slave[0].hasdc && retval > 0)
   // //    {
   // //       ec_sync(ec_DCtime, cycletime, &toff);
   // //    }
   // // }
   // //
   // // rtapi_task_pll_set_correction(toff);
   // //
   // // ec_send_processdata();
   //

   axis_hal_t *axis_data;

   if(firstflag == 0)
   {
      emcecatStatus->motoro = calloc(num_axis, sizeof(Motor_Out*));
      emcecatStatus->motori = calloc(num_axis, sizeof(Motor_In*));

      for (int i = 0; i < num_axis; i++)
      {
         emcecatStatus->motoro[i] = &motorOutInstance[i];
         emcecatStatus->motori[i] = &motorInInstance[i];

         emcecatStatus->motoro[i]->PPtargetPosition = 0;
         emcecatStatus->motori[i]->actualPosition = 0;

         InPos = emcecatStatus->motori[i]->actualPosition;
         emcecatStatus->motoro[i]->PPtargetPosition = InPos;

         atomic_store(&atomic_actpos[i], emcecatStatus->motori[i]->actualPosition);
         atomic_store(&atomic_targetpos[i], emcecatStatus->motoro[i]->PPtargetPosition);

         axes[i].pos_cmd = emcecatStatus->motori[i]->actualPosition;
         axes[i].pos_fb = emcecatStatus->motori[i]->actualPosition;
         axes[i].coarse_pos = emcecatStatus->motori[i]->actualPosition;

         rtapi_print_msg(RTAPI_MSG_INFO, "xis_num %d actualPosition=%d", i, emcecatStatus->motori[i]->actualPosition);
         rtapi_print_msg(RTAPI_MSG_INFO, " PPtargetPosition=%d\n", emcecatStatus->motoro[i]->PPtargetPosition);
      }

      emcmotStatus->carte_pos_cmd.tran.x = axes[0].pos_cmd;
      emcmotStatus->carte_pos_cmd.tran.y = axes[1].pos_cmd;
      emcmotStatus->carte_pos_cmd.tran.z = axes[2].pos_cmd;

      emcmotStatus->carte_pos_fb.tran.x  = axes[0].pos_fb;
      emcmotStatus->carte_pos_fb.tran.y  = axes[1].pos_fb;
      emcmotStatus->carte_pos_fb.tran.z  = axes[2].pos_fb;

      firstflag = 1;
   }

   // rtapi_print_msg(RTAPI_MSG_INFO, "-----------------\n");
   for (int axis_n = 0; axis_n < num_axis; axis_n++ )
   {
      ActPos = atomic_load(&atomic_actpos[axis_n]);
      emcecatStatus->motori[axis_n]->actualPosition = ActPos;
      atomic_store(&atomic_actpos[axis_n], emcecatStatus->motori[axis_n]->actualPosition);

      InPos = atomic_load(&atomic_targetpos[axis_n]);
      emcecatStatus->motoro[axis_n]->PPtargetPosition = InPos;
      atomic_store(&atomic_targetpos[axis_n], emcecatStatus->motoro[axis_n]->PPtargetPosition);

      axes[axis_n].free_tp.curr_pos = emcecatStatus->motori[axis_n]->actualPosition;
      axis_data = &(emcmot_hal_data->axis[axis_n]);
      *(axis_data->motor_pos_fb) = emcecatStatus->motori[axis_n]->actualPosition;

      // if (enprintf % 500 == 0)
      // {
      //    rtapi_print_msg(RTAPI_MSG_INFO, "axis_num %d actpos=%d", axis_n, emcecatStatus->motori[axis_n]->actualPosition);
      //    rtapi_print_msg(RTAPI_MSG_INFO, " targetpos=%d\n", emcecatStatus->motoro[axis_n]->PPtargetPosition);
      // }

      rtapi_print_msg(RTAPI_MSG_INFO, "axis_num %d actpos=%d", axis_n, emcecatStatus->motori[axis_n]->actualPosition);
      rtapi_print_msg(RTAPI_MSG_INFO, " targetpos=%d\n", emcecatStatus->motoro[axis_n]->PPtargetPosition);
   }

   enprintf++;
}

/* Slave error handler */
void *ecatcheck(void *arg)
{
   int slaveix;

   if (dorun)
   {
      if (inOP)
      {
         /* one or more slaves are not responding */
         ec_group[currentgroup].docheckstate = FALSE;
         ec_readstate();
         for (slaveix = 1; slaveix <= 1; slaveix++)
         {
            ec_slavet *slave = &ec_slave[slaveix];

            if ((slave->group == currentgroup) && (slave->state != EC_STATE_OPERATIONAL))
            {
               ec_group[currentgroup].docheckstate = TRUE;
               if (slave->state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
               {
                  rtapi_print_msg(RTAPI_MSG_ERR, "ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slaveix);
                  slave->state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                  ec_writestate(slaveix);
               }
               else if (slave->state == EC_STATE_SAFE_OP)
               {
                  rtapi_print_msg(RTAPI_MSG_WARN, "WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slaveix);
                  slave->state = EC_STATE_OPERATIONAL;
                  ec_writestate(slaveix);
               }
               else if (slave->state > EC_STATE_NONE)
               {
                  if (ec_reconfig_slave(slaveix, EC_TIMEOUTMON) >= EC_STATE_PRE_OP)
                  {
                     slave->islost = FALSE;
                     rtapi_print_msg(RTAPI_MSG_INFO, "MESSAGE : slave %d reconfigured\n", slaveix);
                  }
               }
               else if (!slave->islost)
               {
                  /* re-check state */
                  ec_statecheck(slaveix, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                  if (slave->state == EC_STATE_NONE)
                  {
                     slave->islost = TRUE;
                     /* zero input data for this slave */
                     if (slave->Ibytes)
                     {
                        memset(slave->inputs, 0x00, slave->Ibytes);
                     }
                     rtapi_print_msg(RTAPI_MSG_ERR,"ERROR : slave %d lost\n", slaveix);
                  }
               }
            }
            if (slave->islost)
            {
               if (slave->state <= EC_STATE_INIT)
               {
                  if (ec_recover_slave(slaveix, EC_TIMEOUTMON))
                  {
                     slave->islost = FALSE;
                     rtapi_print_msg(RTAPI_MSG_INFO,"MESSAGE : slave %d recovered\n", slaveix);
                  }
               }
               else
               {
                  slave->islost = FALSE;
                  rtapi_print_msg(RTAPI_MSG_INFO,"MESSAGE : slave %d found\n", slaveix);
               }
            }
         }
         dowkccheck = 0;
      }
   }
   return NULL;
}

/* Transition network to operational state */
int ecatbringup(char *ifname)
{
    if (!ec_init(ifname))
    {
        rtapi_print_msg(RTAPI_MSG_INFO,"初始化网卡失败：%s\n", ifname);
        return -1;
    }
    rtapi_print_msg(RTAPI_MSG_INFO,"初始化网卡成功。\n");
    wkc = ec_config_init(0);
    if (wkc <= 0)
    {
        rtapi_print_msg(RTAPI_MSG_INFO,"未找到任何从站。\n");
        ec_close();
        return -2;
    }
    rtapi_print_msg(RTAPI_MSG_INFO,"检测到 %d 个从站。\n", wkc);

   ec_configdc();

    int64_t cycletime = 1000000;
    for (int i = 1; i <= wkc; i++)
    {
        ec_slave[i].PO2SOconfig = &Motor_PDO_setup;
        ec_dcsync0(i, TRUE, cycletime, cycletime/10);
    }

    ec_config_map(IOmap);
    

    dorun = 1;

    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    /* OP状态请求 */
    ec_writestate(0);
    int chk = 100;
    do
    {
        ec_statecheck(0, EC_STATE_OPERATIONAL, 3000);
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
    } while (ec_slave[0].state != EC_STATE_OPERATIONAL);


    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
    {
        rtapi_print_msg(RTAPI_MSG_INFO,"所有从机OP\n");
    }else{
      return -1;
    }

   inOP = TRUE;

   for (int i = 0; i < wkc; i++)
   {
      emcmot_axis_t *axis;
      axis = &axes[i];
      axis->flag=1;
      *(emcmot_hal_data->axis[i].amp_enable) = 1;
   }

   num_axis = wkc;
   return wkc;
   
}

static uint16_t read_status_word(uint16_t slave_index)
{
    uint16_t status_word = 0x0;
    int retval;
    uint32_t abort_code = 0;
    int data_size = sizeof(status_word);
    
    // 使用SDO读取状态字(0x60641)
    // 使用SDO读取状态字(0x60641)
    retval = ec_SDOread(slave_index, 0x6061, 0x00, FALSE, &data_size, &status_word, EC_TIMEOUTRXM);
    rtapi_print_msg(RTAPI_MSG_INFO, "status_word %04x\n", status_word);
    
    if (retval == 0) {
        rtapi_print_msg(RTAPI_MSG_INFO, "Failed to read status word via SDO from slave %d, error: %d\n", slave_index, retval);
        // 尝试获取详细的错误信息
        ec_SDOread(slave_index, 0x6041, 0x00, TRUE, &data_size, &abort_code, EC_TIMEOUTRXM);
        rtapi_print_msg(RTAPI_MSG_INFO, "Abort code: 0x%08X\n", abort_code);
        return 0xFFFF; // 返回错误值
    }
    
    rtapi_print_msg(RTAPI_MSG_INFO, "Status word via SDO from slave %d: 0x%04X\n", slave_index, status_word);
    return status_word;
}

static void verify_pdo_mapping(uint16_t slave_index)
{
    
    // 读取RxPDO映射(0x1C12)
    uint8_t rx_pdo_count = 0;
    int data_size = sizeof(rx_pdo_count);
    ec_SDOread(slave_index, 0x1C12, 0x00, FALSE, &data_size, &rx_pdo_count, EC_TIMEOUTRXM);
    rtapi_print_msg(RTAPI_MSG_INFO,"RxPDO mapping count: %d\n", rx_pdo_count);
    
    for (int i = 1; i <= rx_pdo_count; i++) {
        uint32_t mapping_entry = 0;
        data_size = sizeof(mapping_entry);
        ec_SDOread(slave_index, 0x1C12, i, FALSE, &data_size, &mapping_entry, EC_TIMEOUTRXM);
        rtapi_print_msg(RTAPI_MSG_INFO,"RxPDO mapping %d: 0x%08X\n", i, mapping_entry);
    }
    
    // 读取TxPDO映射(0x1C13)
    uint8_t tx_pdo_count = 0;
    data_size = sizeof(tx_pdo_count);
    ec_SDOread(slave_index, 0x1C13, 0x00, FALSE, &data_size, &tx_pdo_count, EC_TIMEOUTRXM);
    rtapi_print_msg(RTAPI_MSG_INFO,"TxPDO mapping count: %d\n", tx_pdo_count);
    
    for (int i = 1; i <= tx_pdo_count; i++) {
        uint32_t mapping_entry = 0;
        data_size = sizeof(mapping_entry);
        ec_SDOread(slave_index, 0x1C13, i, FALSE, &data_size, &mapping_entry, EC_TIMEOUTRXM);
        rtapi_print_msg(RTAPI_MSG_INFO,"TxPDO mapping %d: 0x%08X\n", i, mapping_entry);
    }
    
}

int Motor_PDO_setup(uint16 slave)
{
    int retval = 0;
    int sdoret = 0;
    u_int32_t ob2 = 0x0;
    u_int8_t  ob_8bit = 0x0;
    u_int16_t  ob_16bit = 0x0;
    sdoret = ec_SDOwrite(slave, 0x1c12, 0x00, false, sizeof(ob_8bit), &ob_8bit, EC_TIMEOUTRXM);
    retval += sdoret;
    if(sdoret==0)
    {
      rtapi_print_msg(RTAPI_MSG_INFO,"failed to clear Rxpdo mapping, error %d\n", sdoret);
    }

    ob_8bit = 0x0;
    sdoret = ec_SDOwrite(slave, 0x1600, 0x00, false, sizeof(ob_8bit), &ob_8bit, EC_TIMEOUTRXM);
    retval += sdoret;
    if(sdoret==0)
    {
      rtapi_print_msg(RTAPI_MSG_INFO,"failed to clear Rxpdo 0x1600, error %d\n", sdoret);
    }
    ob2 = 0x60400010;
    sdoret = ec_SDOwrite(slave, 0x1600, 0x01, false, sizeof(ob2), &ob2, EC_TIMEOUTRXM);
    retval += sdoret;
    if(sdoret==0)
    {
      rtapi_print_msg(RTAPI_MSG_INFO,"failed to write controlword to rxpdo, error %d\n", sdoret);
    }
    ob2 = 0x60600008;
    sdoret = ec_SDOwrite(slave, 0x1600, 0x02, false, sizeof(ob2), &ob2, EC_TIMEOUTRXM);
    retval += sdoret;
    if(sdoret==0)
    {
      rtapi_print_msg(RTAPI_MSG_INFO,"failed to write targetmode to rxpdo, error %d\n", sdoret);
    }
    ob2 = 0x607A0020;
    sdoret = ec_SDOwrite(slave, 0x1600, 0x03, false, sizeof(ob2), &ob2, EC_TIMEOUTRXM);
    retval += sdoret;
   if(sdoret==0)
    {
      rtapi_print_msg(RTAPI_MSG_INFO,"failed to write targetpos to rxpdo, error %d\n", sdoret);
    }
    ob_8bit = 0x3;
    sdoret = ec_SDOwrite(slave, 0x1600, 0x00, false, sizeof(ob_8bit), &ob_8bit, EC_TIMEOUTRXM);
    retval += sdoret;
    if(sdoret==0)
    {
      rtapi_print_msg(RTAPI_MSG_INFO,"failed to set Rxpdo item, error %d\n", sdoret);
    }

    ob_16bit = 0x1600;
    sdoret = ec_SDOwrite(slave, 0x1c12, 0x01, false, sizeof(ob_16bit), &ob_16bit, EC_TIMEOUTRXM);
    retval += sdoret;
    if(sdoret==0)
    {
      rtapi_print_msg(RTAPI_MSG_INFO,"failed to assign 0x1 to Rxpdo, error %d\n", sdoret);
    }
    ob_8bit = 0x01;
    sdoret = ec_SDOwrite(slave, 0x1c12, 0x00, false, sizeof(ob_8bit), &ob_8bit, EC_TIMEOUTRXM);
    retval += sdoret;
    if(sdoret==0)
    {
      rtapi_print_msg(RTAPI_MSG_INFO,"failed to assign 0x0 to Rxpdo, error %d\n", sdoret);
    }

    ob_8bit = 0x0;
    sdoret = ec_SDOwrite(slave, 0x1c13, 0x00, false, sizeof(ob_8bit), &ob_8bit, EC_TIMEOUTRXM);
    retval += sdoret;
    if(sdoret==0)
    {
      rtapi_print_msg(RTAPI_MSG_INFO,"failed to clear Txpdo mapping, error %d\n", sdoret);
    }
    ob_8bit = 0x0;
    sdoret = ec_SDOwrite(slave, 0x1A00, 0x00, false, sizeof(ob_8bit), &ob_8bit, EC_TIMEOUTRXM);
    retval += sdoret;
    if(sdoret==0)
    {
      rtapi_print_msg(RTAPI_MSG_INFO,"failed to clear Txpdo 0x1A00, error %d\n", sdoret);
    }
    ob2 = 0x60410010;
    sdoret = ec_SDOwrite(slave, 0x1A00, 0x01, false, sizeof(ob2), &ob2, EC_TIMEOUTRXM);
    retval += sdoret;
    if(sdoret==0)
    {
      rtapi_print_msg(RTAPI_MSG_INFO,"failed to write controlword to txpdo, error %d\n", sdoret);
    }
    ob2 = 0x60610008;
    sdoret = ec_SDOwrite(slave, 0x1A00, 0x02, false, sizeof(ob2), &ob2, EC_TIMEOUTRXM);
    retval += sdoret;
    if(sdoret==0)
    {
      rtapi_print_msg(RTAPI_MSG_INFO,"failed to write targetmode to txpdo, error %d\n", sdoret);
    }
    ob2 = 0x60640020;
    sdoret = ec_SDOwrite(slave, 0x1A00, 0x03, false, sizeof(ob2), &ob2, EC_TIMEOUTRXM);
    retval += sdoret;
   if(sdoret==0)
    {
      rtapi_print_msg(RTAPI_MSG_INFO,"failed to write targetpos to txpdo, error %d\n", sdoret);
    }

    ob_8bit = 0x3;
    sdoret = ec_SDOwrite(slave, 0x1A00, 0x00, false, sizeof(ob_8bit), &ob_8bit, EC_TIMEOUTRXM);
    retval += sdoret;

    if(sdoret==0)
    {
      rtapi_print_msg(RTAPI_MSG_INFO,"failed to set txpdo item, error %d\n", sdoret);
    }
    ob_16bit = 0x1A00;
    sdoret = ec_SDOwrite(slave, 0x1c13, 0x01, false, sizeof(ob_16bit), &ob_16bit, EC_TIMEOUTRXM);
    retval += sdoret;
    if(sdoret==0)
    {
      rtapi_print_msg(RTAPI_MSG_INFO,"failed to assign 0x1 to txpdo, error %d\n", sdoret);
    }
    ob_8bit = 0x01;
    sdoret = ec_SDOwrite(slave, 0x1c13, 0x00, false, sizeof(ob_8bit), &ob_8bit, EC_TIMEOUTRXM);
    retval += sdoret;
    if(sdoret==0)
    {
      rtapi_print_msg(RTAPI_MSG_INFO,"failed to assign 0x0 to txpdo, error %d\n", sdoret);
    }

    return 1;
}


static int ecat_module_id;
static int ecat_shmem_id;

int usrecatInit(const char *modname)
{
   int retval;

   ecat_module_id = rtapi_init(modname);
   if (ecat_module_id < 0)
   {
      fprintf(stderr,
          "usrecatInit: ERROR: rtapi init failed\n");
      return -1;
   }
   /* get shared memory block from RTAPI */
   ecat_shmem_id = rtapi_shmem_new(DEFAULT_SHMEM_KEY, ecat_module_id, sizeof(ecat_status_t));
   if (ecat_shmem_id < 0) {
      fprintf(stderr,
          "usrecatInit: ERROR: could not open shared memory\n");
      rtapi_exit(ecat_module_id);
      return -1;
   }
   /* get address of shared memory area */
   retval = rtapi_shmem_getptr(ecat_shmem_id, (void **) &emcecatStruct);
   if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,"usrecatInit: ERROR: could not access shared memory\n");
      rtapi_exit(ecat_module_id);
      return -1;
   }

   return 0;
}

int ecat_thread_main(void)
{
   int retval;

   ecat_comp_id = hal_init("ecatmod");
   if (ecat_comp_id < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,"ECAT: hal_init() failed\n");
      return -1;
   }

   retval = init_ecat_comm_buffers();
   if (retval != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "ECAT: init_ecat_comm_buffers() failed\n");
      hal_exit(ecat_comp_id);
      return -1;
   }

   retval = init_ecat_threads();
   if (retval != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, ("ECAT: init_ecat_threads() failed\n"));
      hal_exit(ecat_comp_id);
      return -1;
   }

   hal_ready(ecat_comp_id);

   rtapi_print_msg(RTAPI_MSG_INFO, "ECAT: ecat_thread init complete\n");

   return 0;
}

int ecat_thread_exit(void)
{
   int retval;

   retval = hal_stop_threads();
   if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
          ("UART: hal_stop_threads() failed, returned %d\n"), retval);
   }
   /* free shared memory */
   retval = rtapi_shmem_delete(ecat_shmem_id, ecat_module_id);
   if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
          ("UART: rtapi_shmem_delete() failed, returned %d\n"), retval);
   }
   /* disconnect from HAL and RTAPI */
   retval = hal_exit(ecat_comp_id);
   if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
          ("UART: hal_exit() failed, returned %d\n"), retval);
   }

   return 0;
}

static int init_ecat_comm_buffers(void)
{
   int retval;
   emcecatStruct = 0;

   retval = rtapi_shmem_getptr(ecat_shmem_id, (void **) &emcecatStruct);
   if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,"MOTION: rtapi_shmem_getptr failed, returned %d\n", retval);
      return -1;
   }

   emcecatStatus = &emcecatStruct->status;
   emcecatConfig = &emcecatStruct->config;

   emcecatStatus->enable = 0;
   emcecatStatus->axstatus = 0;
   emcecatStatus->motype = 0;

   for (int i = 0; i < num_axis; i++)
   {
      *(emcmot_hal_data->axis[i].active) = 1;
   }

   rtapi_print_msg(RTAPI_MSG_INFO, "ect paramed\n");

   return 0;
}

static int init_ecat_threads(void)
{
   long ecat_period_sec = 1000000;
   long check_period_sec = 10000000;
   int retval;

    retval = hal_create_thread("ecat_thread", ecat_period_sec*1000,1,99);
    if (retval < 0) {
       rtapi_print_msg(RTAPI_MSG_ERR,
           "ECAT: failed to create ecat_thread\n");
       return -1;
    }

   //ect status check
   // retval = hal_create_thread("check_thread", check_period_sec,1,94);
   // if (retval < 0) {
   //    rtapi_print_msg(RTAPI_MSG_ERR,
   //        "ECAT: failed to createecat_thread\n");
   //    return -1;
   // }

   //export ecatthread
   retval = hal_export_funct("ecatthread-task", ecatthread, 0	/* arg
    */ , 1 /* uses_fp */ , 0 /* reentrant */ , ecat_comp_id);
   if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
          "MOTION: failed to export usart receive task function\n");
      return -1;
   }

   //export ecatcheck
   // retval = hal_export_funct("ecatcheck-task", ecatcheck, 0	/* arg
   //  */ , 1 /* uses_fp */ , 0 /* reentrant */ , ecat_comp_id);
   // if (retval < 0) {
   //    rtapi_print_msg(RTAPI_MSG_ERR,
   //        "MOTION: failed to export usart process task function\n");
   //    return -1;
   // }

   //add funct to thread
   hal_add_funct_to_thread("ecatthread-task", "ecat_thread", 1);
   // hal_add_funct_to_thread("ecatcheck-task", "check_thread", 1);

   setEcatCycleTime(ecat_period_sec * 1e-9);

   rtapi_print_msg(RTAPI_MSG_INFO, ("ECAT: init_ecat_threads() success\n"));

   return 0;
}


static int setEcatCycleTime(double secs)
{
   /* make sure it's not zero */
   if (secs <= 0.0)
   {
      return -1;
   }

   /* copy into status out */
   emcecatConfig->ecatCycleTime = secs;

   return 0;
}