//
// Created by skeqi on 25-9-10.
//

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include "hpsocket.h"

#include <errno.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include "motion/motion_priv.h"
#include "rtapi/rtapi.h"

//set client
ClientContext clientLinux = {0};
BOOL globalRunning = TRUE;

//set param
RING_BUFF_t TcpRxRingBuff;     // 接收环形缓冲区
sem_t sem_count_tcp_rx;        // 接收消息计数信号量
pthread_mutex_t frame_id_mutex = PTHREAD_MUTEX_INITIALIZER;
uint16_t frame_id = 0;
uint8_t buffer_storage[BUFFER_NUM][BUFFER_SIZE];

extern emcmot_hal_data_t *emcmot_hal_data;
extern emcmot_axis_t axes[EMCMOT_MAX_AXIS];
extern emcmot_command_t *emcmotCommand;
extern emcmot_status_t *emcmotStatus;
extern int num_axis;

static int socket_comp_id;
static socket_struct_t *emcsocketStruct = 0;
static socket_status_t *emcsocketStatus = 0;
static socket_config_t *emcsocketConfig = 0;

static int init_socket_comm_buffers(void);
static int init_socket_threads(void);
static int setSocketCycleTime(double secs);

uint8_t write_ringbuff(RING_BUFF_t *pRingBuf, const linux_frame_t *frame)
{
    pthread_mutex_lock(&pRingBuf->mutex);

    // 检查缓冲区是否已满
    if(pRingBuf->Length >= pRingBuf->buffNum)
    {
        pthread_mutex_unlock(&pRingBuf->mutex);
        rtapi_print_msg(RTAPI_MSG_ERR, "buff is full\n");
        return -1;
    }

    void* write_addr = (uint8_t*)pRingBuf->pBuffer + pRingBuf->Tail * pRingBuf->buffByte;

    if(sizeof(linux_frame_t) > pRingBuf->buffByte) {
        pthread_mutex_unlock(&pRingBuf->mutex);
        rtapi_print_msg(RTAPI_MSG_ERR, "fame size over buffer size\n");
        return -1;
    }

    memcpy(write_addr, frame, sizeof(linux_frame_t));

    pRingBuf->Tail = (pRingBuf->Tail + 1) % pRingBuf->buffNum;
    pRingBuf->Length++;

    pthread_mutex_unlock(&pRingBuf->mutex);
    return 0;
}

uint8_t read_ringbuff(RING_BUFF_t *pRingBuf, linux_frame_t *frame)
{
    pthread_mutex_lock(&pRingBuf->mutex);

    if(pRingBuf->Length == 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "read_ringbuff: Buffer empty\n");
        pthread_mutex_unlock(&pRingBuf->mutex);
        return -1;
    }

    void* read_addr = (uint8_t*)pRingBuf->pBuffer + pRingBuf->Head * pRingBuf->buffByte;

    // 检查读取地址是否有效
    if(read_addr < pRingBuf->pBuffer || read_addr >= (uint8_t*)pRingBuf->pBuffer + pRingBuf->buffNum * pRingBuf->buffByte)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "read_ringbuff: Invalid read address\n");
        pthread_mutex_unlock(&pRingBuf->mutex);
        return -1;
    }

    memcpy(frame, read_addr, sizeof(linux_frame_t));

    pRingBuf->Head = (pRingBuf->Head + 1) % pRingBuf->buffNum;
    pRingBuf->Length--;

    pthread_mutex_unlock(&pRingBuf->mutex);
    return 0;
}


//connect callback
En_HP_HandleResult __stdcall OnConnectLinux(HP_Client pSender, HP_CONNID dwConnID)
{
    clientLinux.bConnected = TRUE;
    clientLinux.frame_state = 0;
    clientLinux.bytes_expected = 0;
    memset(&clientLinux.current_frame, 0, sizeof(tcp_frame_t));
    return HR_OK;
}

//received callback
En_HP_HandleResult __stdcall OnReceiveLinux(HP_Client pSender, HP_CONNID dwConnID, const BYTE* pData, int iLength)
{
    //process
    rtapi_print_msg(RTAPI_MSG_INFO,"receive data\n");
    AnalyseData(pData, iLength);
    return HR_OK;
}


// close callback
En_HP_HandleResult __stdcall OnCloseLinux(HP_Client pSender, HP_CONNID dwConnID, En_HP_SocketOperation enOperation, int iErrorCode)
{
    const char* opStr = "";
    switch(enOperation) {
    case SO_CLOSE: opStr = "关闭"; break;
    case SO_CONNECT: opStr = "连接"; break;
    case SO_ACCEPT: opStr = "接受"; break;
    case SO_SEND: opStr = "发送"; break;
    case SO_RECEIVE: opStr = "接收"; break;
    default: opStr = "未知"; break;
    }

    printf("客户端LAC 连接关闭 [连接ID: %lu, 操作: %s, 错误码: %d]\n", dwConnID, opStr, iErrorCode);
    clientLinux.bConnected = FALSE;
    globalRunning = 0;

    // 安全停止：触发紧急停止命令，防止客户端断开后轴继续运动
    if (emcmotCommand != NULL) {
        emcmotCommand->command = EMCMOT_ABORT;
        emcmotCommand->commandNum++;
    }

    // 唤醒 ProcessTask 线程，防止其永久阻塞在 sem_wait()
    sem_post(&sem_count_tcp_rx);

    return HR_OK;
}

// 初始化客户端
BOOL InitClient(ClientContext* pContext)
{
    // 创建监听器
    pContext->pListener = Create_HP_TcpClientListener();
    if (pContext->pListener == NULL)
    {
        printf("客户端%d 创建监听器失败\n", pContext->id);
        return FALSE;
    }

    // 创建客户端对象
    pContext->pClient = Create_HP_TcpClient(pContext->pListener);
    if (pContext->pClient == NULL)
    {
        printf("客户端%d 创建客户端失败\n", pContext->id);
        Destroy_HP_TcpClientListener(pContext->pListener);
        return FALSE;
    }

    // 设置回调函数
    HP_Set_FN_Client_OnConnect(pContext->pListener, OnConnectLinux);
    HP_Set_FN_Client_OnReceive(pContext->pListener, OnReceiveLinux);
    HP_Set_FN_Client_OnClose(pContext->pListener, OnCloseLinux);

    // 设置其他参数
    HP_TcpClient_SetKeepAliveTime(pContext->pClient, 3000);
    HP_TcpClient_SetKeepAliveInterval(pContext->pClient, 1000);
    // 禁用 Nagle 算法，确保 1ms 运动控制指令不被延迟合并
    HP_TcpClient_SetNoDelay(pContext->pClient, TRUE);

    return TRUE;
}

// 启动客户端
BOOL StartClient(ClientContext* pContext)
{
    if (pContext == NULL) {
        printf("错误：pContext为NULL\n");
        return FALSE;
    }

    if (pContext->pClient == NULL) {
        printf("错误：pClient为NULL\n");
        return FALSE;
    }

    if (pContext->serverIP == NULL) {
        printf("错误：serverIP为NULL\n");
        return FALSE;
    }

    BOOL result = HP_Client_Start(pContext->pClient, pContext->serverIP, pContext->serverPort, FALSE);

    if (!result) {
        int errorCode = HP_Client_GetLastError(pContext->pClient);
        printf("HP_Client_Start失败，错误码: %d\n", errorCode);

        // 根据错误码输出具体信息
        switch (errorCode) {
        case 1: printf("错误：内存分配失败\n"); break;
        case 2: printf("错误：参数无效\n"); break;
        case 3: printf("错误：套接字创建失败\n"); break;
        case 4: printf("错误：绑定失败\n"); break;
        case 5: printf("错误：连接失败\n"); break;
        default: printf("未知错误\n"); break;
        }
        return FALSE;
    }

    printf("客户端%d 已启动，连接到 %s:%d\n", pContext->id, pContext->serverIP, pContext->serverPort);
    pContext->bRunning = TRUE;
    return TRUE;
}

// 停止客户端
void StopClient(ClientContext* pContext)
{
    if (pContext != NULL && pContext->pClient != NULL)
    {
        HP_Client_Stop(pContext->pClient);
        printf("客户端%d 已停止\n", pContext->id);
        pContext->bRunning = FALSE;
        pContext->bConnected = FALSE;
    }
}

// 清理客户端资源
void CleanupClient(ClientContext* pContext)
{
    if (pContext != NULL)
    {
        if (pContext->pClient != NULL)
        {
            Destroy_HP_TcpClient(pContext->pClient);
            pContext->pClient = NULL;
        }

        if (pContext->pListener != NULL)
        {
            Destroy_HP_TcpClientListener(pContext->pListener);
            pContext->pListener = NULL;
        }

        printf("客户端%d 资源已清理\n", pContext->id);
    }
}

int socket_init()
{
    // connect server
    // 初始化客户端
    clientLinux.id = 1;
    clientLinux.serverIP = "192.168.1.1";
    clientLinux.serverPort = 55655;
    clientLinux.bRunning = FALSE;
    clientLinux.bConnected = FALSE;
    clientLinux.frame_state = 0;

    // 初始化分包解析器
    clientLinux.parser.state = FRAME_STATE_IDLE;
    clientLinux.parser.received_len = 0;
    clientLinux.parser.expected_len = 0;

    // 初始化无锁 SPSC 队列
    clientLinux.spsc_q.write_idx = 0;
    clientLinux.spsc_q.read_idx = 0;

    // 检查IP地址是否有效
    if (clientLinux.serverIP == NULL || strlen(clientLinux.serverIP) == 0) {
        printf("错误：服务器IP地址为空\n");
        return -1;
    }

    // 检查端口是否有效
    if (clientLinux.serverPort <= 0 || clientLinux.serverPort > 65535) {
        printf("错误：端口号无效 %d\n", clientLinux.serverPort);
        return -1;
    }

    if (!InitClient(&clientLinux))
    {
        printf("Init clientLAC failed\n");
        return -1;
    }

    // 启动客户端
    if (!StartClient(&clientLinux) )
    {
        printf("Start client failed\n");
        CleanupClient(&clientLinux);
        return -1;
    }

    return 0;
}

En_HP_HandleResult AnalyseData(BYTE* pData,int iLength)
{
    if (pData == NULL || iLength < 10)
    { // 最小帧长度：帧头4 + 数据长度2 + 校验和4 = 10字节
        rtapi_print_msg(RTAPI_MSG_ERR, "Invalid data: null pointer or length too short\n");
    }

    // 帧解析状态机：处理粘包/半包
    for (int i = 0; i < iLength; i++) {
        uint8_t byte = pData[i];

        switch (clientLinux.parser.state) {
        case FRAME_STATE_IDLE:
            // 查找帧头 0x55AA55AA (小端: 0x55 0xAA 0x55 0xAA)
            clientLinux.parser.rx_buf[clientLinux.parser.received_len++] = byte;
            if (clientLinux.parser.received_len >= 4) {
                uint32_t header = *(uint32_t*)clientLinux.parser.rx_buf;
                if (header == 0x55AA55AA) {
                    clientLinux.parser.state = FRAME_STATE_GOT_HEADER;
                    clientLinux.parser.received_len = 0;
                } else {
                    // 帧头不匹配，滑动窗口：丢弃第一个字节，从第二个字节重新查找
                    memmove(clientLinux.parser.rx_buf, clientLinux.parser.rx_buf + 1, clientLinux.parser.received_len - 1);
                    clientLinux.parser.received_len--;
                }
            }
            break;

        case FRAME_STATE_GOT_HEADER:
            clientLinux.parser.rx_buf[clientLinux.parser.received_len++] = byte;
            if (clientLinux.parser.received_len >= 2) {
                clientLinux.parser.expected_len = *(uint16_t*)(clientLinux.parser.rx_buf);
                // 帧头(4) + 数据长度(2) + 数据 + 校验和(4) <= 256
                uint16_t total_frame_len = 4 + 2 + clientLinux.parser.expected_len + 4;
                if (clientLinux.parser.expected_len > 128 || total_frame_len > 256) {
                    rtapi_print_msg(RTAPI_MSG_ERR, "Invalid data length: %d\n", clientLinux.parser.expected_len);
                    clientLinux.parser.state = FRAME_STATE_IDLE;
                    clientLinux.parser.received_len = 0;
                } else {
                    clientLinux.parser.state = FRAME_STATE_GOT_LENGTH;
                }
            }
            break;

        case FRAME_STATE_GOT_LENGTH: {
            clientLinux.parser.rx_buf[4 + clientLinux.parser.received_len++] = byte;
            uint16_t total_needed = 6 + clientLinux.parser.expected_len + 4;
            if (clientLinux.parser.received_len >= clientLinux.parser.expected_len + 4) {
                // 收到完整帧，开始校验
                uint16_t data_len = clientLinux.parser.expected_len;
                uint32_t calculated_checksum = 0;
                // 校验和范围：帧头后 2 字节(data_length) + 载荷数据
                for (int j = 4; j < 4 + data_len; j++) {
                    calculated_checksum += clientLinux.parser.rx_buf[j];
                }
                // 注意：客户端用 uint (4字节) 发送校验和，此处也用 uint32_t 对齐
                uint32_t received_checksum = *(uint32_t*)(clientLinux.parser.rx_buf + 4 + data_len);

                if (calculated_checksum != received_checksum) {
                    rtapi_print_msg(RTAPI_MSG_ERR, "Checksum error: calc=0x%08X, recv=0x%08X\n",
                                   calculated_checksum, received_checksum);
                    clientLinux.parser.state = FRAME_STATE_IDLE;
                    clientLinux.parser.received_len = 0;
                    break;
                }

                // 解析帧有效载荷
                clientLinux.current_frame.header = 0x55AA55AA;
                clientLinux.current_frame.data_length = data_len;
                clientLinux.current_frame.checksum = calculated_checksum;
                memcpy(clientLinux.current_frame.payload, clientLinux.parser.rx_buf + 6, data_len);

                // 使用无锁 SPSC 队列（替代 mutex 环冲区，避免优先级反转）
                if (spsc_push(&clientLinux.spsc_q, &clientLinux.current_frame) != 0) {
                    rtapi_print_msg(RTAPI_MSG_ERR, "SPSC queue full, frame dropped\n");
                } else {
                    sem_post(&sem_count_tcp_rx);
                }

                clientLinux.parser.state = FRAME_STATE_IDLE;
                clientLinux.parser.received_len = 0;
            }
            break;
        }

        default:
            clientLinux.parser.state = FRAME_STATE_IDLE;
            clientLinux.parser.received_len = 0;
            break;
        }
    }
    return HR_OK;
}

// 处理任务线程
void* ProcessTask(void* arg)
{
    emcmot_axis_t *axis;
    linux_frame_t frame;

    // 看门狗超时设置：1 秒无指令则报警（防止连接断开后永久阻塞）
    struct timespec ts;

    while (globalRunning)
    {
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 1;  // 1 秒超时

        int ret = sem_timedwait(&sem_count_tcp_rx, &ts);
        if (ret == -1) {
            // ETIMEDOUT 或其他错误，继续循环检查 globalRunning
            continue;
        }

        // 使用无锁 SPSC 队列读取
        if (spsc_pop(&clientLinux.spsc_q, &frame) == 0)
        {
            uint16_t axis_index, cmd_index;
            memcpy(&cmd_index, frame.payload, sizeof(uint16_t));
            memcpy(&axis_index, frame.payload + 2, sizeof(uint16_t));
            cmd_code_t cmd_type = (cmd_code_t)cmd_index;

            uint8_t *param_data = frame.payload + 4;
            size_t param_data_length = frame.data_length - 4;

            switch(cmd_type)
            {
                case EMCMOT_DOWNLOADS_CONFIG:
                    if (param_data_length >= sizeof(AxisParam))
                    {
                        AxisParam *param = (AxisParam *)param_data;

                        if (emcmotCommand)
                        {
                            emcmotCommand->command = cmd_type;
                            emcmotCommand->commandNum++;
                            emcmotCommand->pulse = param->AxisPulse;
                            emcmotCommand->Maxacc = param->AxisMaxAcc;
                            emcmotCommand->Maxdec = param->AxisMaxDec;
                            emcmotCommand->Maxvel = param->AxisMaxVel;
                            emcmotCommand->acc = param->AxisAcc;
                            emcmotCommand->dec = param->AxisDec;
                            emcmotCommand->vel = param->AxisVel;
                            emcmotCommand->scale = param->AxisVelScale;
                            emcmotCommand->maxAxisScale = param->AxisMaxVelScale;
                            emcmotCommand->maxLimit = param->AxisLimitPlus;
                            emcmotCommand->minLimit = param->AxisLimitMinus;
                            emcmotCommand->axis = axis_index;
                        }
                    }
                    else
                    {
                        rtapi_print_msg(RTAPI_MSG_ERR,"error:  AxisParam Incomplete\n");
                    }
                    break;
                case EMCMOT_AXIS_ENABLE:
                    if (param_data_length >= sizeof(AxisEnableParam))
                    {
                        AxisEnableParam *param = (AxisEnableParam *)param_data;
                        rtapi_print_msg(RTAPI_MSG_INFO,"start axis enable\n");
                        if (emcmotCommand)
                        {
                            *(emcmot_hal_data->enable) = 1;
                            emcmotCommand->command = cmd_type;
                            emcmotCommand->commandNum++;
                            emcmotCommand->axis = axis_index;
                        }
                    }
                    break;

                case EMCMOT_AXIS_DISABLE:
                    if (param_data_length >= sizeof(AxisDisableParam))
                    {
                        AxisDisableParam *param = (AxisDisableParam *)param_data;
                        rtapi_print_msg(RTAPI_MSG_INFO,"start axis disable\n");
                        if (emcmotCommand)
                        {
                            *(emcmot_hal_data->enable) = 0;
                            emcmotCommand->command = cmd_type;
                            emcmotCommand->commandNum++;
                            emcmotCommand->axis = axis_index;
                        }
                    }
                    break;

                case EMCMOT_MOTION_ENABLE:
                    if (param_data_length >= sizeof(MotionEnableParam))
                    {
                        MotionEnableParam *param = (MotionEnableParam *)param_data;
                        rtapi_print_msg(RTAPI_MSG_INFO,"start motion enable\n");
                        if (emcmotCommand)
                        {
                            emcmotCommand->command = cmd_type;
                            emcmotCommand->commandNum++;
                            emcmotCommand->axis = axis_index;
                        }
                    }
                    break;

                case EMCMOT_MOTION_DISABLE:
                    if (param_data_length >= sizeof(MotionDisableParam))
                    {
                        MotionDisableParam *param = (MotionDisableParam *)param_data;
                        rtapi_print_msg(RTAPI_MSG_INFO,"start motion disable\n");
                        if (emcmotCommand)
                        {
                            emcmotCommand->command = cmd_type;
                            emcmotCommand->commandNum++;
                            emcmotCommand->axis = axis_index;
                        }
                    }
                    break;

                case EMCMOT_JOG_CONT:
                    if (param_data_length >= sizeof(AxisContJogParam))
                    {
                        AxisContJogParam *param = (AxisContJogParam *)param_data;
                        axis = &axes[axis_index];
                        if (emcmotCommand)
                        {
                            axis->acc_cmd = param->JogAcc;
                            axis->vel_cmd = param->JogVel;
                            axis->acc_limit = emcmotCommand->Maxacc;
                            axis->vel_limit = emcmotCommand->Maxvel;
                            axis->max_pos_limit = emcmotCommand->maxLimit;
                            axis->min_pos_limit = emcmotCommand->minLimit;
                            axis->max_jog_limit = emcmotCommand->maxLimit;
                            axis->min_jog_limit = emcmotCommand->minLimit;
                            emcmotCommand->dec = param->JogDec;
                            emcmotCommand->dir = param->JogDir;
                            emcmotCommand->commandNum++;
                            emcmotCommand->command = cmd_type;
                            emcmotCommand->axis = axis_index;
                        }
                    }
                break;

                case EMCMOT_JOG_INCR:
                    if (param_data_length >= sizeof(AxisRelJogParam))
                    {
                        AxisRelJogParam *param = (AxisRelJogParam *)param_data;
                        axis = &axes[axis_index];
                        if (emcmotCommand)
                        {
                            axis->acc_cmd = param->JogAcc;
                            axis->vel_cmd = param->JogVel;
                            axis->acc_limit = emcmotCommand->Maxacc;
                            axis->vel_limit = emcmotCommand->Maxvel;
                            axis->max_pos_limit = emcmotCommand->maxLimit;
                            axis->min_pos_limit = emcmotCommand->minLimit;
                            axis->max_jog_limit = emcmotCommand->maxLimit;
                            axis->min_jog_limit = emcmotCommand->minLimit;
                            emcmotCommand->dec = param->JogDec;
                            emcmotCommand->dir = param->JogDir;
                            emcmotCommand->offset = param->JogCmdPos;
                            emcmotCommand->commandNum++;
                            emcmotCommand->command = cmd_type;
                            emcmotCommand->axis = axis_index;
                        }
                    }
                    break;
                case EMCMOT_JOG_ABS:
                    if (param_data_length >= sizeof(AxisAbsJogParam))
                    {
                        AxisAbsJogParam *param = (AxisAbsJogParam *)param_data;
                        axis = &axes[axis_index];
                        if (emcmotCommand)
                        {
                            axis->acc_cmd = param->JogAcc;
                            axis->vel_cmd = param->JogVel;
                            axis->acc_limit = emcmotCommand->Maxacc;
                            axis->vel_limit = emcmotCommand->Maxvel;
                            axis->max_pos_limit = emcmotCommand->maxLimit;
                            axis->min_pos_limit = emcmotCommand->minLimit;
                            axis->max_jog_limit = emcmotCommand->maxLimit;
                            axis->min_jog_limit = emcmotCommand->minLimit;
                            emcmotCommand->dec = param->JogDec;
                            emcmotCommand->offset = param->JogCmdPos;
                            emcmotCommand->commandNum++;
                            emcmotCommand->command = cmd_type;
                            emcmotCommand->axis = axis_index;
                        }
                    }
                    break;

                case EMCMOT_JOG_ABORT:
                    if (param_data_length >= sizeof(AxisStopParam))
                    {
                        AxisStopParam *param = (AxisStopParam *)param_data;
                        rtapi_print_msg(RTAPI_MSG_INFO,"stop axis motion\n");
                        if (emcmotCommand)
                        {
                            emcmotCommand->command = cmd_type;
                            emcmotCommand->commandNum++;
                            emcmotCommand->axis = axis_index;
                        }
                    }
                    break;

                case EMCMOT_SET_LINE:
                    if (param_data_length >= sizeof(LineaInterpParam))
                    {
                        LineaInterpParam *param = (LineaInterpParam *)param_data;
                        rtapi_print_msg(RTAPI_MSG_INFO,"start linear motion\n");
                        if (emcmotCommand)
                        {
                            for (int i = 0; i < EMCMOT_MAX_AXIS; i++)
                            {
                                axis = &axes[i];
                                if (param->AxisIndex[i] == 1)
                                {
                                    axis->acc_cmd = param->LinearAcc;
                                    axis->vel_cmd = param->LinearVel;
                                    axis->acc_limit = emcmotCommand->Maxacc;
                                    axis->vel_limit = emcmotCommand->Maxvel;
                                    axis->max_pos_limit = emcmotCommand->maxLimit;
                                    axis->min_pos_limit = emcmotCommand->minLimit;
                                    axis->max_jog_limit = emcmotCommand->maxLimit;
                                    axis->min_jog_limit = emcmotCommand->minLimit;
                                }

                            }
                            emcmotCommand->pos.tran.x = (double)param->LinearPos[0];
                            emcmotCommand->pos.tran.y = (double)param->LinearPos[1];
                            emcmotCommand->pos.tran.z = (double)param->LinearPos[2];

                            emcmotCommand->ini_maxvel = param->LinearStartVel;
                            emcmotCommand->vel = param->LinearVel;
                            emcmotCommand->acc = param->LinearAcc;
                            emcmotCommand->dec = param->LinearDec;
                            emcmotCommand->motion_type = EMC_MOTION_TYPE_FEED;
                            emcmotCommand->dir = param->MoveDir;
                            emcmotCommand->ref = param->ReferenceDir;
                            emcmotCommand->command = cmd_type;
                            emcmotCommand->commandNum++;
                            emcmotCommand->axis = axis_index;
                            emcmotCommand->spindle = axis_index;
                        }
                    }
                    break;

                case EMCMOT_SET_FREE:
                    if (param_data_length >= sizeof(FreeMotionParam))
                    {
                        FreeMotionParam *param = (FreeMotionParam *)param_data;
                        rtapi_print_msg(RTAPI_MSG_INFO,"start axis free\n");
                        if (emcmotCommand)
                        {
                            emcmotCommand->command = cmd_type;
                            emcmotCommand->commandNum++;
                            emcmotCommand->axis = axis_index;
                        }
                    }
                    break;

                case EMCMOT_SET_COORD:
                    if (param_data_length >= sizeof(CoordMotionParam))
                    {
                        CoordMotionParam *param = (CoordMotionParam *)param_data;
                        rtapi_print_msg(RTAPI_MSG_INFO,"start axis coord\n");
                        if (emcmotCommand)
                        {
                            emcmotCommand->command = cmd_type;
                            emcmotCommand->commandNum++;
                            emcmotCommand->axis = axis_index;
                        }
                    }
                    break;

                default:
                    rtapi_print_msg(RTAPI_MSG_ERR, "error; not cmd %d\n", cmd_type);
                    break;
            }
        }
    }

    return NULL;
}

// 压缩轴数据
uint16_t compress_axis_data(axis_hal_t *axis_data, uint8_t *buffer)
{
    uint16_t offset = 0;

    union {
        int32_t int_value;
        uint8_t bytes[4];
    } converter;

    const int32_t *int32_fields[] = {
        axis_data->axis_vel_cmd,
        axis_data->axis_acc_cmd,
        axis_data->axis_dec_cmd,
        axis_data->axis_pos_cmd,
        axis_data->axis_pos_fb,
        axis_data->f_error,
        axis_data->axis_motion,
        axis_data->axis_status,
    };

    const int int32_field_count = sizeof(int32_fields) / sizeof(int32_fields[0]);

    for (int i = 0; i < int32_field_count; i++) {
        if (int32_fields[i] != NULL) {
            converter.int_value = (int32_t)*int32_fields[i];
        } else {
            converter.int_value = 0;
        }
        memcpy(buffer + offset, converter.bytes, 4);
        offset += 4;
    }

    // 状态位
    uint16_t status_bits = 0;
    if (*(axis_data->amp_enable)) status_bits |= 0x0001;
    if (*(axis_data->active)) status_bits |= 0x0002;
    if (*(axis_data->motionenable)) status_bits |= 0x0004;
    if (*(axis_data->motionrunning)) status_bits |= 0x0008;
    if (*(axis_data->f_errored)) status_bits |= 0x0010;
    if (*(axis_data->in_position)) status_bits |= 0x0020;
    if (*(axis_data->amp_fault)) status_bits |= 0x0040;
    if (*(axis_data->pos_lim_sw)) status_bits |= 0x0080;
    if (*(axis_data->neg_lim_sw)) status_bits |= 0x0100;
    if (*(axis_data->phl)) status_bits |= 0x0200;
    if (*(axis_data->nhl)) status_bits |= 0x0400;

    memcpy(buffer + offset, &status_bits, 2);
    offset += 2;

    return offset;
}

uint16_t compress_multiple_axes_data(axis_hal_t axes_data[], int axis_count, uint8_t *buffer)
{
    if (axes_data == NULL || buffer == NULL || axis_count <= 0) {
        return 0;
    }

    uint16_t total_offset = 0;

    for (int i = 0; i < axis_count; i++) {
        uint16_t axis_data_size = compress_axis_data(&axes_data[i], buffer + total_offset);
        total_offset += axis_data_size;
    }

    return total_offset;
}

uint16_t calculate_checksum(tcp_frame_t *frame)
{
    uint16_t sum = 0;

    // 计算payload数据
    for (int i = 0; i < frame->data_length; i++) {
        sum += frame->payload[i];
    }

    return sum;
}

void* ClientThreadData(void* arg)
{
    if (num_axis <= 0)
    {
        return NULL;
    }

    if (clientLinux.bRunning && globalRunning)
    {
        tcp_frame_t frame;
        // 准备轴数据帧
        frame.header = 0xAA55AA55;

        pthread_mutex_lock(&frame_id_mutex);
        if (emcmot_hal_data)
        {
            axis_hal_t temp_axis[num_axis];
            for (int i = 0; i < num_axis; i++ )
            {
                temp_axis[i] = emcmot_hal_data->axis[i];
            }
            frame.data_length = compress_multiple_axes_data(temp_axis, num_axis, frame.payload);
        } else {
            return NULL;
        }
        pthread_mutex_unlock(&frame_id_mutex);

        // 计算校验和
        frame.checksum = calculate_checksum(&frame);

        //send
        SendFrame(&clientLinux, &frame);
    }
}

// 发送帧
BOOL SendFrame(ClientContext* pContext, tcp_frame_t* frame)
{
    if (pContext == NULL || pContext->pClient == NULL || !pContext->bConnected)
    {
        return FALSE;
    }

    size_t frame_length = sizeof(frame->header) +
                          sizeof(frame->data_length) +
                          frame->data_length +
                          sizeof(frame->checksum);

    return HP_Client_Send(pContext->pClient, (const BYTE*)frame, frame_length);
}

static int socket_module_id;
static int socket_shmem_id;

int usrsocketInit(const char *modname)
{
    int retval;

    socket_module_id = rtapi_init(modname);
    if (socket_module_id < 0)
    {
        return -1;
    }
    /* get shared memory block from RTAPI */
    socket_shmem_id = rtapi_shmem_new(DEFAULT_SHMEM_KEY, socket_module_id, sizeof(socket_status_t));
    if (socket_shmem_id < 0) {
        rtapi_exit(socket_module_id);
        return -1;
    }
    /* get address of shared memory area */
    retval = rtapi_shmem_getptr(socket_shmem_id, (void **) &emcsocketStruct);
    if (retval < 0) {
        rtapi_exit(socket_module_id);
        return -1;
    }

    return 0;
}

int socket_thread_main(void)
{
    int retval;

    if (0 != usrsocketInit("emc_task3")) {
        return -1;
    }

    socket_comp_id = hal_init("socketmod");
    if (socket_comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_INFO,"SOCKET hal_init() failed\n");
        return -1;
    }

    retval = init_socket_comm_buffers();
    if (retval != 0) {
        rtapi_print_msg(RTAPI_MSG_INFO,"SOCKET: init_socket_comm_buffers() failed\n");
        hal_exit(socket_comp_id);
        return -1;
    }

    retval = init_socket_threads();
    if (retval != 0) {
        rtapi_print_msg(RTAPI_MSG_INFO,"SOCKET: init_socket_threads() failed\n");
        hal_exit(socket_comp_id);
        return -1;
    }

    hal_ready(socket_comp_id);

    return 0;
}

int socket_thread_exit(void)
{
    int retval;

    retval = hal_stop_threads();
    if (retval < 0) {
    }
    /* free shared memory */
    retval = rtapi_shmem_delete(socket_shmem_id, socket_module_id);
    if (retval < 0) {
    }
    /* disconnect from HAL and RTAPI */
    retval = hal_exit(socket_comp_id);
    if (retval < 0) {
    }

    return 0;
}

//初始化uart线程相关参数
static int init_socket_comm_buffers(void)
{
    int retval;
    emcsocketStruct = 0;

    // 初始化接收环形缓冲区
    TcpRxRingBuff.Head = 0;
    TcpRxRingBuff.Tail = 0;
    TcpRxRingBuff.Length = 0;
    TcpRxRingBuff.buffNum = BUFFER_NUM;
    TcpRxRingBuff.buffByte = BUFFER_SIZE*2;
    TcpRxRingBuff.pBuffer = buffer_storage;
    pthread_mutex_init(&TcpRxRingBuff.mutex, NULL);

    // 初始化信号量和互斥锁
    sem_init(&sem_count_tcp_rx, 0, 0);

    retval = rtapi_shmem_getptr(socket_shmem_id, (void **) &emcsocketStruct);
    if (retval < 0) {
        return -1;
    }

    emcsocketStatus = &emcsocketStruct->status;
    emcsocketConfig = &emcsocketStruct->config;

    return 0;
}

static int init_socket_threads(void)
{
    double socket_period_sec = 1000000;
    int retval;

    retval = hal_create_thread("client_p", socket_period_sec*10,1,96);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_INFO,"SOCKET: failed to create client_p\n");
        return -1;
    }

    retval = hal_create_thread("client_s", socket_period_sec*1000*5,1,95);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_INFO,"SOCKET: failed to create client_s\n");
        return -1;
    }

    retval = hal_export_funct("ProcessTask", ProcessTask, 0	/* arg
     */ , 1 /* uses_fp */ , 0 /* reentrant */ , socket_comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_INFO,"SOCKET: failed to export ProcessTask function\n");
        return -1;
    }

    retval = hal_export_funct("ClientThreadData", ClientThreadData, 0	/* arg
     */ , 1 /* uses_fp */ , 0 /* reentrant */ , socket_comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_INFO,"SOCKET: failed to export ClientThreadData function\n");
        return -1;
    }

    hal_add_funct_to_thread("ClientThreadData", "client_s", 1);
    hal_add_funct_to_thread("ProcessTask", "client_p", 1);


    setSocketCycleTime(socket_period_sec * 1e-9);

    return 0;
}

static int setSocketCycleTime(double secs)
{
    /* make sure it's not zero */
    if (secs <= 0.0) {
        return -1;
    }

    /* copy into status out */
    emcsocketConfig->socketCycleTime = secs;

    return 0;
}