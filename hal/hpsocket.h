//
// Created by skeqi on 25-9-10.
//

#ifndef HPSOCKET_H
#define HPSOCKET_H

#include <string.h>
#include "stdint.h"
#include "motion/motion.h"
#include "motion/motion_priv.h"
#include "rtapi/rtapi_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BUFFER_NUM 64
#define BUFFER_SIZE 128


#include "hpsocket/include/HPSocket4C.h"

    typedef enum {
        DISCONNECT = 0,
        CONNECT,
        MONITORING
    } SOCKET_STATUS_t;

#pragma pack(push)
#pragma pack(1)
    typedef struct {
        uint32_t header;         //帧头标识 0xAA55AA55
        uint16_t data_length;     // 数据部分长度
        uint8_t payload[68];     // 数据负载
        uint16_t checksum;        // 校验和
    }tcp_frame_t;

#pragma pack(pop)

#pragma pack(push)
#pragma pack(1)
    typedef struct {
        uint32_t header;         //帧头标识 0xAA55AA55
        uint16_t data_length;     // 数据部分长度
        uint8_t payload[128];     // 数据负载
        uint32_t checksum;        // 校验和
    }linux_frame_t;
#pragma pack(pop)

#pragma pack(push)
#pragma pack(1)
    //环形缓冲区
    typedef struct 
    {
        pthread_mutex_t mutex;
        uint8_t Head;
        uint8_t Tail;
        uint8_t Length;
        uint32_t buffNum;
        uint32_t buffByte;
        void* pBuffer;
    } RING_BUFF_t;
#pragma pack(pop)

// 无锁 SPSC 队列（替代 mutex 环冲区）
// 仅两个线程访问：网络线程(生产者) + 实时线程(消费者)
// 无需 mutex，利用内存屏障保证顺序
#define SPSC_QUEUE_SIZE 64

    typedef struct {
        volatile uint32_t write_idx;  // 仅生产者(网络线程)写入
        volatile uint32_t read_idx;   // 仅消费者(实时线程)读取
        linux_frame_t slots[SPSC_QUEUE_SIZE];
    } spsc_queue_t;

    static inline uint32_t spsc_count(spsc_queue_t* q) {
        return q->write_idx - q->read_idx;
    }

    static inline int spsc_push(spsc_queue_t* q, const linux_frame_t* frame) {
        uint32_t next = (q->write_idx + 1) % SPSC_QUEUE_SIZE;
        if (next == q->read_idx) return -1;  // 队列满
        memcpy(&q->slots[q->write_idx], frame, sizeof(linux_frame_t));
        __sync_synchronize();                  // 内存屏障
        q->write_idx = next;
        return 0;
    }

    static inline int spsc_pop(spsc_queue_t* q, linux_frame_t* frame) {
        if (q->read_idx == q->write_idx) return -1;  // 队列空
        __sync_synchronize();                          // 内存屏障
        memcpy(frame, &q->slots[q->read_idx], sizeof(linux_frame_t));
        q->read_idx = (q->read_idx + 1) % SPSC_QUEUE_SIZE;
        return 0;
    }

    // 帧解析状态机（处理粘包/半包）
    typedef enum {
        FRAME_STATE_IDLE = 0,
        FRAME_STATE_GOT_HEADER,
        FRAME_STATE_GOT_LENGTH,
        FRAME_STATE_COMPLETE
    } frame_parse_state_t;

    typedef struct {
        frame_parse_state_t state;
        uint16_t           expected_len;
        uint16_t           received_len;
        uint8_t            rx_buf[256];
    } frame_parser_t;

#pragma pack(push)
#pragma pack(1)
    // 客户端结构体
    typedef struct {
        int id;
        HP_TcpClient pClient;
        HP_TcpClientListener pListener;
        BOOL bRunning;
        BOOL bConnected;
        const char* serverIP;
        int serverPort;
        pthread_t threadId;
        uint8_t frame_state;
        uint16_t bytes_expected;
        linux_frame_t current_frame;
        uint32_t temp_checksum;
        frame_parser_t parser;           // 粘包/半包解析器
        spsc_queue_t   spsc_q;           // 无锁 SPSC 队列
    } ClientContext;
#pragma pack(pop)

#pragma pack(push)
#pragma pack(1)

    typedef struct {
        int32_t AxisPulse;
        int32_t AxisStartVel;
        int32_t AxisVel;
        int32_t AxisEndVel;
        int32_t AxisVelScale;
        int32_t AxisAcc;
        int32_t AxisDec;
        int32_t AxisMaxVel;
        int32_t AxisMaxAcc;
        int32_t AxisMaxDec;
        int32_t AxisMaxVelScale;
        int32_t AxisLimit;
        int32_t AxisLimitPlus;
        int32_t AxisLimitMinus;
    } AxisParam;

    typedef struct {
        int32_t AxisEnable;
    } AxisEnableParam;

    typedef struct {
        int32_t AxisEnable;
    } AxisDisableParam;

    typedef struct {
        int32_t MotionEnable;
    } MotionEnableParam;

    typedef struct {
        int32_t MotionDisable;
    } MotionDisableParam;

    typedef struct {
        int32_t JogVel;
        int32_t JogAcc;
        int32_t JogDec;
        int32_t JogDir;
    } AxisContJogParam;

    typedef struct {
        int32_t JogVel;
        int32_t JogStartVel;
        int32_t JogAcc;
        int32_t JogDec;
        int32_t JogCmdPos;
        int32_t JogDir;
    } AxisRelJogParam;

    typedef struct {
        int32_t JogVel;
        int32_t JogStartVel;
        int32_t JogAcc;
        int32_t JogDec;
        int32_t JogCmdPos;
    } AxisAbsJogParam;

    typedef struct
    {
        int32_t LinearStartVel;
        int32_t LinearVel;
        int32_t LinearAcc;
        int32_t LinearDec;
        int32_t ReferenceDir;
        int32_t MoveDir;
        int32_t LinearPos[EMCMOT_MAX_AXIS];
        int32_t AxisIndex[EMCMOT_MAX_AXIS];
    }LineaInterpParam;

    typedef struct
    {
        int32_t CycleStartVel;
        int32_t CycleVel;
        int32_t CycleAcc;
        int32_t CycleDec;
        int32_t Turn;
        int32_t AxisIndex[EMCMOT_MAX_AXIS];
        int32_t Normal[EMCMOT_MAX_AXIS];
        int32_t CycleEndPos[EMCMOT_MAX_AXIS];
        int32_t CycleEnterPos[EMCMOT_MAX_AXIS];
    }CycleInterpParam;

    typedef struct {
        int32_t FreeMotion;
    } FreeMotionParam;

    typedef struct {
        int32_t CoordMotion;
    } CoordMotionParam;

    typedef struct {
        int32_t AxisStop;
    } AxisStopParam;

    #pragma pack(pop)

    typedef struct socket_status_t
    {
        SOCKET_STATUS_t socket_status;
        cmd_code_t client_cmd;
        RING_BUFF_t buff_data;
    }socket_status_t;

    typedef struct socket_config_t
    {
        double socketCycleTime;

    }socket_config_t;

    typedef struct socket_struct_t {
        rtapi_mutex_t command_mutex;
        struct socket_status_t status;
        struct socket_config_t config;
    }socket_struct_t;


#pragma pack(pop)

    uint16_t calculate_checksum(tcp_frame_t *frame);
    uint8_t write_ringbuff(RING_BUFF_t *pRingBuf, const linux_frame_t *frame);
    uint8_t read_ringbuff(RING_BUFF_t *pRingBuf, linux_frame_t *frame);

    En_HP_HandleResult __stdcall OnConnectLinux(HP_Client pSender, HP_CONNID dwConnID);
    En_HP_HandleResult __stdcall OnReceiveLinux(HP_Client pSender, HP_CONNID dwConnID, const BYTE* pData, int iLength);
    En_HP_HandleResult __stdcall OnCloseLinux(HP_Client pSender, HP_CONNID dwConnID, En_HP_SocketOperation enOperation, int iErrorCode);

    En_HP_HandleResult AnalyseData(BYTE* pData,int iLength);
    BOOL SendFrame(ClientContext* pContext, tcp_frame_t* frame);
    BOOL InitClient(ClientContext* pContext);
    BOOL StartClient(ClientContext* pContext);
    void StopClient(ClientContext* pContext);
    void CleanupClient(ClientContext* pContext);
    void* ProcessTask(void* arg);
    uint16_t compress_axis_data(axis_hal_t *axis_data, uint8_t *buffer);
    uint16_t compress_multiple_axes_data(axis_hal_t axes_data[], int axis_count, uint8_t *buffer);
    void* ClientThreadData(void* arg);
    extern int socket_init();
    int usrsocketInit(const char *modname);
    int socket_thread_main(void);
    int socket_thread_exit(void);

#ifdef __cplusplus
}
#endif

#endif //HPSOCKET_H
