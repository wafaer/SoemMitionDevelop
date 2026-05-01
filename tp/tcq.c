#include "tcq.h"
#include <stddef.h>

/*
 * tcqCheck — 队列有效性检查
 */
static inline int tcqCheck(TC_QUEUE_STRUCT const * const tcq)
{
    if ((0 == tcq) || (0 == tcq->queue))
    {
        return -1;
    }
    return 0;
}

/*
 * tcqCreate — 创建并初始化轨迹段队列
 */
int tcqCreate(TC_QUEUE_STRUCT * const tcq, int _size, TC_STRUCT * const tcSpace)
{
    if (!tcq || !tcSpace || _size < 1) {
        return -1;
    }
	tcq->queue = tcSpace;
	tcq->size = _size;
    tcqInit(tcq);

	return 0;
}

/*
 * tcqDelete — 删除队列
 */
int tcqDelete(TC_QUEUE_STRUCT * const tcq)
{
    if (!tcqCheck(tcq)) {
        tcq->queue = 0;
    }

    return 0;
}

/*
 * tcqInit — 初始化队列为空状态
 */
int tcqInit(TC_QUEUE_STRUCT * const tcq)
{
    if (tcqCheck(tcq)) return -1;

    tcq->_len = 0;
    tcq->start = tcq->end = 0;
    tcq->rend = 0;
    tcq->_rlen = 0;
    tcq->allFull = 0;

    return 0;
}

/*
 * tcqPut — 将 TC 放入队尾
 */
int tcqPut(TC_QUEUE_STRUCT * const tcq, TC_STRUCT const * const tc)
{
    if (tcqCheck(tcq)) return -1;

    /* 检查 allFull，如果队列已满则拒绝写入 */
    if (tcq->allFull) {
	    return -1;
    }

    /* 将 TC 复制到队尾位置 */
    tcq->queue[tcq->end] = *tc;
    tcq->_len++;

    /* 更新队尾指针，环形回绕 */
    tcq->end = (tcq->end + 1) % tcq->size;

    /* 如果队尾追上队首，说明队列已满 */
    if (tcq->end == tcq->start) {
	    tcq->allFull = 1;
    }

    return 0;
}

/*
 * tcqPopBack — 弹出队尾（反向操作）
 */
int tcqPopBack(TC_QUEUE_STRUCT * const tcq)
{
    if (tcqCheck(tcq)) return -1;

    if (tcq->_len < 1) {
        return -1;
    }

    /* 环形减法：end 前移一个位置
     * 加 size 再取模，避免 end=0 时的负数问题 */
    int n = tcq->end - 1 + tcq->size;
    tcq->end = n % tcq->size;
    tcq->_len--;

    return 0;
}

/* TCQ_REVERSE_MARGIN — 反向历史的最大容量
 */
#define TCQ_REVERSE_MARGIN 200

/*
 * tcqPop — 从队首弹出元素（正向执行）
 */
int tcqPop(TC_QUEUE_STRUCT * const tcq)
{

    if (tcqCheck(tcq)) {
        return -1;
    }

    /* 队列必须有元素才能弹出，或者如果是满的（allFull=1）也算有元素 */
    if (tcq->_len < 1 && !tcq->allFull) {
        return -1;
    }

    /* 更新队首指针，清除 allFull 标志 */
    tcq->start = (tcq->start + 1) % tcq->size;
    tcq->allFull = 0;
    tcq->_len--;

    /* 更新反向历史长度 */
    if (tcq->_rlen < TCQ_REVERSE_MARGIN) {
        tcq->_rlen++;
    } else {
        tcq->rend = (tcq->rend + 1) % tcq->size;
    }

    return 0;
}

/*
 * tcqRemove — 从队首移除 n 个元素
 */
int tcqRemove(TC_QUEUE_STRUCT * const tcq, int n)
{

    if (n <= 0) {
	    return 0;
    }

    /* 检查队列有效性和可移除性 */
    if (tcqCheck(tcq) || ((tcq->start == tcq->end) && !tcq->allFull) ||
            (n > tcq->_len)) {	/* too many requested */
	    return -1;
    }

    /* 更新队首指针，清除 allFull 标志 */
    tcq->start = (tcq->start + n) % tcq->size;
    tcq->allFull = 0;
    tcq->_len -= n;

    return 0;
}


/**
 * tcqBackStep — 反向执行一步（将段退回队列）
 */
int tcqBackStep(TC_QUEUE_STRUCT * const tcq)
{

    if (tcqCheck(tcq)) {
        return -1;
    }

    /* 如果 start == rend，说明反向历史为空 */
    if ( tcq->start == tcq->rend) {
        return -1;
    }
    /* 更新队首指针（后退一步） */
    tcq->start = (tcq->start - 1 + tcq->size) % tcq->size;
    tcq->_len++;
    tcq->_rlen--;

    return 0;
}

/*
 * tcqLen — 获取队列长度
 */
int tcqLen(TC_QUEUE_STRUCT const * const tcq)
{
    if (tcqCheck(tcq)) return -1;

    return tcq->_len;
}

/*
 * tcqItem — 获取第 n 个元素（不删除）
 */
TC_STRUCT * tcqItem(TC_QUEUE_STRUCT const * const tcq, int n)
{
    if (tcqCheck(tcq) || (n < 0) || (n >= tcq->_len)) return NULL;

    /* 环形索引计算：(start + n) % size */
    return &(tcq->queue[(tcq->start + n) % tcq->size]);
}

/* TC_QUEUE_MARGIN — 队列"接近满"的判断边界
 */
#define TC_QUEUE_MARGIN (TCQ_REVERSE_MARGIN+20)

/*
 * tcqFull — 判断队列是否"满"
 */
int tcqFull(TC_QUEUE_STRUCT const * const tcq)
{
    if (tcqCheck(tcq)) {
	   return 1;
    }

    /* 如果队列太小（size <= TC_QUEUE_MARGIN），直接返回 allFull */
    if (tcq->size <= TC_QUEUE_MARGIN) {
	    return tcq->allFull;
    }

    /* 如果 _len 进入边界区域（>= size - TC_QUEUE_MARGIN），认为已满 */
    if (tcq->_len >= tcq->size - TC_QUEUE_MARGIN) {
	    return 1;
    }

    /* 未进入边界区域 */
    return 0;
}

/*
 * tcqLast — 获取队尾元素（最新加入的）
 */
TC_STRUCT *tcqLast(TC_QUEUE_STRUCT const * const tcq)
{
    if (tcqCheck(tcq)) {
        return NULL;
    }
    if (tcq->_len == 0) {
        return NULL;
    }
    /* end - 1 + size 避免 end=0 时的负数取模 */
    int n = tcq->end-1 + tcq->size;
    return &(tcq->queue[n % tcq->size]);
}
