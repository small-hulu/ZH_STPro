/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Neck.c
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
 * @attention
  *
  * Belongs to neck test for ZEHO AI ROBOT Demo V1.0.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* USER CODE BEGIN */
#ifndef __MOTOR_H
#define __MOTOR_H

#include <string.h>
#include "Neck.h"
#include "tim.h"
#include "servo_angles.h"
/**
* @brief  Twist The Neck
* @param  Angle
*         This parameter can be one of the following values:
*            @arg Angle: Servo_R_MAX(LEFT MAX)
*            @arg Angle: Servo_R_MIN(RIGHT MAX)
*            @arg Angle: Servo_R_MID(Start position)
* @retval cmd status
*	 Position ------------------------------------------------------------------
*																		 back
*																		Servo_3
*																		
*																		Servo_R
*																		
*														Servo_1						Servo_2
*0:0.5ms-50   45:1ms-100	90:1.5ms-150		135:2ms-200			180:2.5ms-250
 */
/*
void NeckTwist(uint8_t Angle)
{
	if((Angle>Servo_R_MAX) || (Angle<Servo_R_MIN))
	{
		return;
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Angle);
	}
}*/

/**
  * @brief  Twist The Neck
  * @param  Angle
  *         This parameter can be one of the following values:
  *            @arg Angle: Servo_R_MAX(LEFT MAX)
  *            @arg Angle: Servo_R_MIN(RIGHT MAX)
  *            @arg Angle: Servo_R_MID(Start position)
  * @retval cmd status

  */
int16_t neck_r_cur_angle = 90; 
int16_t neck_r_target_angle = 90;

int16_t neck_pitch_cur = 0;      
int16_t neck_pitch_target = 0;   

uint16_t neck1_ccr = 100;
uint16_t neck2_ccr = 100;
uint16_t neck3_ccr = 100;
void Neck_Pitch_SetTargetAngle(int16_t angle)
{
    if (angle < 0)  angle = 0;
    if (angle > 60) angle = 60;

    neck_pitch_target = angle;
}
void Neck_Pitch_Update(void)
{
    if (neck_r_cur_angle < neck_r_target_angle)
        neck_r_cur_angle++;
    else if (neck_r_cur_angle > neck_r_target_angle)
        neck_r_cur_angle--;

    if (neck_pitch_cur < neck_pitch_target)
        neck_pitch_cur++;
    else if (neck_pitch_cur > neck_pitch_target)
        neck_pitch_cur--;

    uint16_t r_ccr = neck_angle_to_ccr(neck_r_cur_angle);

    neck1_ccr = CLAMP(100 + neck_pitch_cur, NECK_1_MIN, NECK_1_MAX);
    neck2_ccr = CLAMP(100 - neck_pitch_cur, NECK_2_MIN, NECK_2_MAX);
    neck3_ccr = CLAMP(100 + neck_pitch_cur, NECK_3_MIN, NECK_3_MAX);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, r_ccr);  // 水平
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, neck1_ccr);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, neck2_ccr);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, neck3_ccr);
}

void TestNod()
{
	Neck_R_SetTargetAngle(120);
	
//	for(uint8_t i=0;i<=20;i++){
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, NECK_R_MID+i);
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 100+i);
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 100-i);
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 100+i);
//		HAL_Delay(40);
//	}
//	for(uint8_t i=0;i<=20;i++){
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 170-i);
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 120-i);
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 80+i);
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 120-i);
//		HAL_Delay(40);
//	}
}


// 计算数组长度
uint16_t len = ARRAY_LENGTH(servo1_angles);
uint16_t cur_ccr;
void Neckinit(void)
{
     cur_ccr = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, NECK_R_MID);

    for(int i=0;i<=30;i++){
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 70+i);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 70+i);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 70+i);
        HAL_Delay(40);
    }
}

void NeckNod()
{
	for(uint8_t i=0;i<=30;i++){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 100+i);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 100-i);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 100+i);
		HAL_Delay(20);
	}

	for(uint8_t i=0;i<=30;i++){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 130-i);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 70+i);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 130-i);
		HAL_Delay(20);
	}
}

void NeckUpDown()
{
	for(uint8_t i=0;i<=15;i++){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 95-i);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 90-i*2/3);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 95-i);
		HAL_Delay(50);
	}
	for(int i=0;i<=15;i++){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 80+i);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 80+i*2/3);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 80+i);
		HAL_Delay(50);
	}
}
void NeckShow(){
	for(uint8_t i=0;i<len;i++){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 50+(servo1_angles[i]-30)/0.9);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 50+(servo2_angles[i]-30)/0.9);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 50+(servo3_angles[i]-30)/0.9);
		HAL_Delay(20);
	}
}

//angle_to_ccr    0~180° -> ccr:50 ~250
uint16_t neck_angle_to_ccr(int16_t angle)
{
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;

    return NECK_R_MIN + angle * (NECK_R_MAX - NECK_R_MIN) / 180;
}

void Neck_R_SetTargetAngle(int16_t angle)
{
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;

    neck_r_target_angle = angle;
}

void Neck_R_Update(void)
{
    if (neck_r_cur_angle == neck_r_target_angle)
        return;

    if (neck_r_cur_angle < neck_r_target_angle)
        neck_r_cur_angle++;
    else
        neck_r_cur_angle--;

    uint16_t ccr = neck_angle_to_ccr(neck_r_cur_angle);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
}


/* USER CODE END */
#endif

