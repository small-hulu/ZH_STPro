/*
 * act.c
 *
 *  Created on: Dec 18, 2025
 *      Author: ZEHO
 */
#include "act.h"
#include "motor.h"
#include "StepMotor.h"
#include "tim.h"
#include <math.h>

MotorFeedback_t fb1, fb2;
/**********************************************************轮毂电机控制*********************************************************/
/************************初始化**********************/
void Car_Init(){
	Motor_Init(&huart2);

	//第一次开启 DMA+IDLE 接收
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, xUART2.RxTemp, sizeof(xUART2.RxTemp));// 开启DMA空闲中断
	/* 上电后延时稳定 */
	HAL_Delay(300);

	/* 使能 + 切速度环模式 + 设置转速 */
	Motor_Enable(1);
	HAL_Delay(50);
	Motor_Enable(2);
	HAL_Delay(50);
	Motor_SetMode(1, MOTOR_MODE_SPEED);
	HAL_Delay(50);
	Motor_SetMode(2, MOTOR_MODE_SPEED);
	HAL_Delay(50);
}
/************************前进************************/
void GoAhead(){
	//前进
	Motor_SetSpeed(1, 2540, 25);
	Motor_SetSpeed(2, -2540, 25);
}
/************************减速************************/
void SlowDown(){
	//减速
	Motor_SetSpeed(1, 50, 50);
	Motor_SetSpeed(2, -50, 50);
}
/************************后退************************/
void GoBack(){
	Motor_SetSpeed(1, -50, 25);
	Motor_SetSpeed(2, 50, 25);
}
/************************右转************************/
void TurnRight(){
	Motor_SetSpeed(1, 50, 0);
	Motor_SetSpeed(2, 50, 0);
}
/************************左转************************/
void TurnLeft(){
	Motor_SetSpeed(1, -50, 0);
	Motor_SetSpeed(2, -50, 0);
}
/************************停止************************/
void Stop(){
	Motor_SetSpeed(1, 0, 25);
	Motor_SetSpeed(2, 0, 25);
}
/************************获取反馈信息************************/
void FeedBack(){
	Motor_GetFeedback(&fb1, 1);
	Motor_GetFeedback(&fb2, 2);
}

/**
 * @brief 接收 ROS 速度并驱动电机
 * @param cmd  ROS 下发速度
 * @param acc  加速度参数（0~255）
 */
int16_t rpm_l,rpm_r;
float v_left,v_right;
#define MS_TO_RPM (60.0f / (2.0f * M_PI * WHEEL_RADIUS_M))

void Motor_SetSpeed_FromCmd(const cmd_vel_t *cmd, uint8_t acc)
{
    float linear_x_m_s = cmd->linear_x_mm_s / 1000.0f;
    float angular_z_rad_s = cmd->angular_z_mrad / 1000.0f;

    v_left  = linear_x_m_s - angular_z_rad_s * (WHEEL_BASE_M / 2.0f);
    v_right = linear_x_m_s + angular_z_rad_s * (WHEEL_BASE_M / 2.0f);

    rpm_l = (int16_t)(v_left  * MS_TO_RPM) *10;
    rpm_r = (-(int16_t)(v_right * MS_TO_RPM)) * 10;


    Motor_SetSpeed(1, rpm_l, acc);
    Motor_SetSpeed(2, rpm_r, acc);
}

/**********************************************************翅膀电机控制*********************************************************/
void ArmWave()//挥手
{
	Stepper_SetSpeed(5);

	Stepper_RotateAngle1(15, STEPPER_BACKWARD);//翅中摆动
	Stepper_RotateAngle2(15, STEPPER_FORWARD);//翅中摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
	Stepper_RotateAngle1(30, STEPPER_FORWARD);//翅中摆动
	Stepper_RotateAngle2(30, STEPPER_BACKWARD);//翅中摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
	Stepper_RotateAngle1(30, STEPPER_BACKWARD);//翅中摆动
	Stepper_RotateAngle2(30, STEPPER_FORWARD);//翅中摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
	Stepper_RotateAngle1(15, STEPPER_FORWARD);//翅中摆动
	Stepper_RotateAngle2(15, STEPPER_BACKWARD);//翅中摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
}
void ArmGive()//伸手
{
	Stepper_SetSpeed(5);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);//左翅根摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1000);//右翅根摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
	osDelay(400);
	Stepper_RotateAngle1(15, STEPPER_BACKWARD);//翅中摆动
	Stepper_RotateAngle2(15, STEPPER_FORWARD);//翅中摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
}
void ArmRaise()//举手
{
	Stepper_SetSpeed(5);

	Stepper_RotateAngle1(15, STEPPER_BACKWARD);//翅中摆动
	Stepper_RotateAngle2(15, STEPPER_FORWARD);//翅中摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
}
/*************************上下扑棱***************************/
void ArmUpDown(){
	Stepper_SetSpeed(10);

	Stepper_RotateAngle1(5, STEPPER_FORWARD);//左翅中摆动
	Stepper_RotateAngle2(5, STEPPER_FORWARD);//右翅中摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
	Stepper_RotateAngle1(10, STEPPER_BACKWARD);//左翅中摆动
	Stepper_RotateAngle2(10, STEPPER_BACKWARD);//右翅中摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
	Stepper_RotateAngle1(10, STEPPER_FORWARD);//左翅中摆动
	Stepper_RotateAngle2(10, STEPPER_FORWARD);//右翅中摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
	Stepper_RotateAngle1(5, STEPPER_BACKWARD);//左翅中摆动
	Stepper_RotateAngle2(5, STEPPER_BACKWARD);//右翅中摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(200);
}
/**********************************前后扑棱*********************************/
void ArmFrontBack(){
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);//左翅根摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 2000);//右翅根摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
	HAL_Delay(100);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);//左翅根摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);//右翅根摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
	HAL_Delay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 2000);//左翅根摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);//右翅根摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 2000);
	HAL_Delay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);//左翅根摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);//右翅根摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
	HAL_Delay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);//左翅根摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 2000);//右翅根摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
	HAL_Delay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);//左翅根摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);//右翅根摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
	HAL_Delay(200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 2000);//左翅根摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);//右翅根摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 2000);
	HAL_Delay(100);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);//左翅根摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);//右翅根摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
}

void Armidle1()//次子
{
	int rand(void);     // 生成伪随机数
	void srand(unsigned int seed); // 设置随机种子
	uint32_t delay_ms = (rand() % 3 + 4) * 1000;  // 1000 / 2000 / 3000 ms
	
	Stepper_SetSpeed(5);

	Stepper_RotateAngle1(12, STEPPER_BACKWARD);//左翅中摆动
	osDelay(delay_ms);
	Stepper_RotateAngle1(7, STEPPER_FORWARD);//左翅中摆动
}
void ArmRaise1()//举手1次子
{
	Stepper_SetSpeed(5);

	Stepper_RotateAngle1(22, STEPPER_BACKWARD);//左翅中摆动

}
void ArmRaise2()//举手次子
{

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(80);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(10);

	osDelay(10);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1000);
	osDelay(145);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(10);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(80);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);//左翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);//右翅尖摆动
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 2000);
	osDelay(10);
}
void ArmRaise3()//举手3次子
{
	Stepper_SetSpeed(5);

	Stepper_RotateAngle1(17, STEPPER_FORWARD);//左翅中摆动

}



