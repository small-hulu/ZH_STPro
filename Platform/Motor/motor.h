#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "usart.h"

/* 命令 */
#define MOTOR_CMD_MODE    0xA0
#define MOTOR_CMD_RUN     0x64
#define MOTOR_CMD_READ    0xA1   // M0603A 的反馈读取命令实际是 A1
#define MOTOR_CMD_MIL     0x74   //里程计信息

/* 模式 */
#define MOTOR_MODE_ENABLE  0x08
#define MOTOR_MODE_DISABLE 0x09
#define MOTOR_MODE_OPEN    0x00
#define MOTOR_MODE_CURR    0x01   // 电流（转矩）模式
#define MOTOR_MODE_SPEED   0x02   // 速度模式
#define MOTOR_MODE_POS     0x03   // 位置模式（部分驱动支持）

typedef struct {
    int16_t speed;
    int16_t current;
    uint8_t temperature;
    uint8_t fault;
} MotorFeedback_t;

typedef struct
{
    int32_t lap_count;   // 里程圈数（有符号）
    uint16_t position;  // 0~32767 -> 0~360°
    uint8_t fault;
} MotorExtFeedback_t;

static inline float Motor_PositionToDegree(uint16_t pos)
{
    return (pos / 32767.0f) * 360.0f;
}

void Motor_Init(UART_HandleTypeDef *huart);

void Motor_Enable(uint8_t id);
void Motor_Disable(uint8_t id);

void Motor_SetMode(uint8_t id, uint8_t mode);

void Motor_SetSpeed(uint8_t id, int16_t rpm, uint8_t acc);
void Motor_SetTorque(uint8_t id, int16_t ma, uint8_t acc);
void Motor_SetPosition(uint8_t id, int32_t pulses, uint8_t acc);

void Motor_RequestFeedback(uint8_t id);

uint8_t Motor_GetFeedback(MotorFeedback_t *fb, uint8_t id);
void Motor_ProcessRxData(uint8_t *buf, uint16_t len);
void Motor_RequestExtFeedback(uint8_t id);
uint8_t Motor_GetExtFeedback(MotorExtFeedback_t *fb, uint8_t id);


#endif
