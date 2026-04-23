#ifndef SERVO_PRIV_H
#define SERVO_PRIV_H

typedef uint32_t u_int32_t;
typedef uint16_t u_int16_t;
typedef uint8_t  u_int8_t;

//axis
#pragma pack(push)
#pragma pack(1)

typedef struct Motor_In {
    u_int16_t statusword;     //6041  状态字
    int8_t    workModeIn;     //6061  工作模式
    int32_t   actualPosition; // 6064 实际位置
}Motor_In;

// PDO映射 电机模块输入控制结构体
typedef struct  Motor_Out {
    u_int16_t controlword;      // 6040  control字
    int8_t    workModeOut;     //6060   工作模式
    int32_t  PPtargetPosition; //607a CSP 目标位置
}Motor_Out;

#pragma pack(pop)

#endif /* SERVO_PRIV_H */
