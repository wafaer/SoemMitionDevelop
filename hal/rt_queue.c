//
// Created by Administrator on 2025/8/30.
//

#include "rt_queue.h"

void pool_init(NodePool *pool)
{
    pool->freeList = NULL;
#if THREAD_SAFE
    pthread_mutex_init(&pool->lock, NULL);
#endif
    for (int i = 0; i < POOL_SIZE; i++){
        pool->nodes[i].next = pool->freeList;
        pool->freeList = &pool->nodes[i];
    }
}

Node* pool_alloc(NodePool *pool) {
    LOCK(pool->lock);
    Node *node = pool->freeList;
    if (node) {
        pool->freeList = node->next;
    }
    UNLOCK(pool->lock);
    return node;
}

void pool_free(NodePool *pool, Node *node) {
    LOCK(pool->lock);
    node->next = pool->freeList;
    pool->freeList = node;
    UNLOCK(pool->lock);
}

void queue_init(Queue *q, NodePool *pool) {
    q->front = q->rear = NULL;
    q->pool = pool;
#if THREAD_SAFE
    pthread_mutex_init(&q->lock, NULL);
#endif
}

int queue_isEmpty(Queue *q) {
    return q->front == NULL;
}

int enqueue(Queue *q, void *data) {
    Node *node = pool_alloc(q->pool);
    if (!node) {
        printf("内存池空，无法入队\n");
        return 0;
    }
    node->data = data;
    node->next = NULL;

    LOCK(q->lock);
    if (q->rear) {
        q->rear->next = node;
        q->rear = node;
    } else {
        q->front = q->rear = node;
    }
    UNLOCK(q->lock);
    return 1;
}

int dequeue(Queue *q, void **data) {
    LOCK(q->lock);
    if (!q->front) {
        UNLOCK(q->lock);
        return 0;
    }
    Node *node = q->front;
    *data = node->data;
    q->front = node->next;
    if (!q->front) q->rear = NULL;
    UNLOCK(q->lock);

    pool_free(q->pool, node);
    return 1;
}