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

/*
 * 【外部共享内存指针】
 * 这些指针指向由 motion.c 在模块加载时分配的共享内存区域。
 * extern 关键字表示这些变量在别处定义（motion.c），此处仅声明引用。
 *
 * 【指向关系】
 *   motion.c 创建 emcmotStruct（包含所有子结构）并映射到共享内存。
 *   用户空间通过 usrmotInit() 获取映射地址，填充这些全局指针。
 *   本文件中的所有读写操作都通过这些指针进行。
 */
extern emcmot_command_t *emcmotCommand;   /* 指向共享内存中的命令缓冲区 */
extern emcmot_status_t *emcmotStatus;    /* 指向共享内存中的状态缓冲区 */
extern emcmot_config_t *emcmotConfig;    /* 指向共享内存中的配置缓冲区 */
extern emcmot_internal_t *emcmotInternal; /* 指向共享内存中的内部状态缓冲区 */
extern emcmot_error_t *emcmotError;      /* 指向共享内存中的错误环形缓冲区 */
extern emcmot_struct_t *emcmotStruct;    /* 指向包含所有缓冲区的顶层结构 */

/*
 * usrmotWriteEmcmotCommand — 向运动控制模块发送命令
 */
int usrmotWriteEmcmotCommand(emcmot_command_t * c)
{
    emcmot_status_t s;
    /* commandNum : 命令序号，静态局部变量 */
    static int commandNum = 0;
    /* end : 超时截止时间（绝对时间戳） */
    double end;

    /* 【参数校验 1】检查 motion ID 是否有效
     * MOTION_ID_VALID(x) 定义为 (x) != MOTION_INVALID_ID
     * MOTION_INVALID_ID = INT_MIN
     * 某些轨迹命令需要分配一个唯一 ID 用于跟踪，
     * 如果传入无效 ID（如 MOTION_INVALID_ID），直接拒绝。 */
    if (!MOTION_ID_VALID(c->id)) {
        rtapi_print_msg(RTAPI_MSG_ERR, "USRMOT: ERROR: invalid motion id: %d\n",c->id);
        return EMCMOT_COMM_INVALID_MOTION_ID;
    }

    /* 【连接检查】确认共享内存中的命令缓冲区是否已映射
     * 如果 emcmotCommand 为空指针，说明共享内存尚未初始化或已被卸载。 */
    if (0 == emcmotCommand) {
        rtapi_print_msg(RTAPI_MSG_ERR,"USRMOT: ERROR: can't connect to shared memory\n");
        return EMCMOT_COMM_ERROR_CONNECT;
    }

    /* 写入命令到共享内存 */
    rtapi_mutex_get(&emcmotStruct->command_mutex);
    *emcmotCommand = *c;
    rtapi_mutex_give(&emcmotStruct->command_mutex);

    /*  写入命令后，需要等待实时线程处理完成并更新状态。
     * 使用"序号匹配 + 状态检查"协议。 */

    /* 设置超时截止时间 = 当前时间 + 默认超时时长（DEFAULT_EMCMOT_COMM_TIMEOUT = 1.0 秒） */
    end = etime() + DEFAULT_EMCMOT_COMM_TIMEOUT;
    /* 轮询循环：只要当前时间未超过截止时间，就持续检查 */
    while (etime() < end) {
        /* 读取最新状态（传入局部变量 s，而非直接使用全局指针）。
         * usrmotReadEmcmotStatus() 会处理分裂读取问题（见其详细注释）。 */
        if (( usrmotReadEmcmotStatus(&s) == 0 ) && ( s.commandNumEcho == commandNum )) {
            /* 【序号匹配成功】
             * 实时线程已处理了本命令（commandNumEcho 回显了我们的序号）。
             * 现在检查命令的执行结果。 */
            if (s.commandStatus == EMCMOT_COMMAND_OK) {
                /* 命令执行成功 */
                return EMCMOT_COMM_OK;
            } else {
                /* 命令被拒绝或执行失败
                 * （例如参数无效、当前状态不允许此命令等） */
                rtapi_print_msg(RTAPI_MSG_ERR, "USRMOT: ERROR: invalid command\n");
                return EMCMOT_COMM_ERROR_COMMAND;
            }
        }
        /* 序号尚未匹配或读取失败，短暂休眠后重试
         * esleep(25e-6) = 休眠 25 微秒
         * 不使用忙等待（busy-wait），让出 CPU 给其他线程 */
        esleep(25e-6);
    }
    /* 超时处理 */
    rtapi_print_msg(RTAPI_MSG_ERR, "USRMOT: ERROR: command %u timeout (seq: %d)\n", c->command, commandNum);
    return EMCMOT_COMM_ERROR_TIMEOUT;
}

/*
 * usrmotReadEmcmotStatus — 从共享内存读取运动状态
 */
int usrmotReadEmcmotStatus(emcmot_status_t * s)
{
    /* split_read_count : 分裂读取重试计数器 */
    int split_read_count;

    /* 【连接检查】确认共享内存中的状态缓冲区是否已映射 */
    if (0 == emcmotStatus) {
        return EMCMOT_COMM_ERROR_CONNECT;
    }
    split_read_count = 0;
    do {
        /* 第一次迭代后，休眠 1 微秒（让出 CPU）。
         * 注意：第一次读取不需要休眠（因为调用者可能刚刚轮询完）。 */
        if(split_read_count > 0) esleep(1e-6);	// Don't busy-loop and give time to process
        /* 【核心读取操作】
         * 将共享内存中的状态完整复制到用户空间局部变量 s。
         * sizeof(emcmot_status_t) 确保整个结构体被复制。
         * memcpy 是按字节复制，不依赖任何锁（本函数外部由调用者保证一致性）。 */
        memcpy(s, emcmotStatus, sizeof(emcmot_status_t));
        /* 【一致性检查】
         * 检查 head 和 tail 是否相等。
         * - 相等：数据一致（全旧或全新），读取成功。
         * - 不等：发生了分裂读取（读写者并发），需要重试。 */
        if (s->head == s->tail) {
            return EMCMOT_COMM_OK;
        }
    } while ( ++split_read_count < 3 );

    return EMCMOT_COMM_SPLIT_READ_TIMEOUT;
}

/*
 * usrmotReadEmcmotConfig — 从共享内存读取配置参数
 */
int usrmotReadEmcmotConfig(emcmot_config_t * s)
{
    int split_read_count;

    if (0 == emcmotConfig) {
	return EMCMOT_COMM_ERROR_CONNECT;
    }
    split_read_count = 0;
    do {
	if(split_read_count > 0) esleep(1e-6);
	memcpy(s, emcmotConfig, sizeof(emcmot_config_t));
	if (s->head == s->tail) {
	    return EMCMOT_COMM_OK;
	}
    } while ( ++split_read_count < 3 );
    return EMCMOT_COMM_SPLIT_READ_TIMEOUT;
}

/* usrmotReadEmcmotInternal — 从共享内存读取内部状态
 */
int usrmotReadEmcmotInternal(emcmot_internal_t * s)
{
    int split_read_count;

    if (0 == emcmotInternal) {
	return EMCMOT_COMM_ERROR_CONNECT;
    }
    split_read_count = 0;
    do {
	if(split_read_count > 0) esleep(1e-6);
	memcpy(s, emcmotInternal, sizeof(emcmot_internal_t));
	if (s->head == s->tail) {
	    return EMCMOT_COMM_OK;
	}
    } while ( ++split_read_count < 3 );
    return EMCMOT_COMM_SPLIT_READ_TIMEOUT;
}
