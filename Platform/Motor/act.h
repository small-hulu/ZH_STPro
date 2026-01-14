/*
 * act.h
 *
 *  Created on: Dec 18, 2025
 *      Author: ZEHO
 */

#ifndef INC_ACT_H_
#define INC_ACT_H_

#include <stdint.h>

typedef struct
{
	 uint16_t  RxNum;            // 接收字节数，只要字节数>0，即为接收到新一帧数据
	 uint8_t   RxData[512];     // 接收到的数据（对外使用）
	 uint8_t   RxTemp[512];    // 临时缓存（对内作接收），在DMA空闲中断中将把其内容复制到RxData[ ]
} xUART_TypeDef;
/**
 * ROS 下发的线速度 & 角速度
 * 单位：
 *   linear_x : mm/s
 *   angular_z: mrad/s
 */
typedef struct {
    int16_t linear_x_mm_s;   // mm/s
    int16_t angular_z_mrad;  // mrad/s（或你定义的单位）
} cmd_vel_t;


/* ====== 机器人参数 ====== */
#define WHEEL_RADIUS_M 0.0375f
#define WHEEL_BASE_M   0.40f

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

extern xUART_TypeDef xUART2;

#define STEPPER_FORWARD  0
#define STEPPER_BACKWARD 1
void ArmWave();
void ArmGive();
void ArmRaise();
void Car_Init();
void GoAhead();
void SlowDown();
void GoBack();
void TurnRight();
void TurnLeft();
void Stop();
void FeedBack();

void ArmUpDown();
void ArmFrontBack();
void Motor_SetSpeed_FromCmd(const cmd_vel_t *cmd, uint8_t acc);
#endif /* INC_ACT_H_ */
