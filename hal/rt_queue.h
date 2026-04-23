//
// Created by Administrator on 2025/8/30.
//

#ifndef RT_QUEUE_H
#define RT_QUEUE_H

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#define POOL_SIZE 1024   // 内存池节点数量
#define THREAD_SAFE 1    // 1:线程安全，0:非线程安全

#if THREAD_SAFE
#define LOCK(mutex)   pthread_mutex_lock(&mutex)
#define UNLOCK(mutex) pthread_mutex_unlock(&mutex)
#else
#define LOCK(mutex)
#define UNLOCK(mutex)
#endif

typedef struct Node {
    void *data;
    struct Node *next;
} Node;

typedef struct {
    Node nodes[POOL_SIZE];
    Node *freeList;
#if THREAD_SAFE
    pthread_mutex_t lock;
#endif
} NodePool;

typedef struct {
    Node *front;
    Node *rear;
    NodePool *pool;
#if THREAD_SAFE
    pthread_mutex_t lock;
#endif
} Queue;


extern void pool_init(NodePool *pool);

extern Node* pool_alloc(NodePool *pool);

extern void pool_free(NodePool *pool, Node *node);

extern void queue_init(Queue *q, NodePool *pool);

extern int queue_isEmpty(Queue *q);

extern int enqueue(Queue *q, void *data);

extern int dequeue(Queue *q, void **data);



#endif //RT_QUEUE_H
