/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Neck.h
  * @brief          : Header for Neck.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Belongs to neck test for ZEHO AI ROBOT Demo V1.0.
  * All rights reserved.
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* USER CODE BEGIN */
/* Position ------------------------------------------------------------------
*																		 back
*																		Servo_3
*																		
*																		Servo_R
*																		
*														Servo_1						Servo_2
*																			face
*timer: ARR=1999  -50Hz
*0:0.5ms-50   45:1ms-100	90:1.5ms-150		135:2ms-200			180:2.5ms-250
*/
#ifndef INC_NECK_H_
#define INC_NECK_H_

#include "stm32f4xx_hal.h"

#ifndef CLAMP
#define CLAMP(x, min, max)  ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#endif

extern int16_t neck_r_cur_angle;
extern int16_t neck_r_target_angle;
extern int16_t neck_pitch_cur;    
extern int16_t neck_pitch_target;  

extern uint16_t neck1_ccr;
extern uint16_t neck2_ccr;
extern uint16_t neck3_ccr;

typedef enum {
    NECK_PITCH_0   = 0,
    NECK_PITCH_20  = 20,
    NECK_PITCH_40  = 40,
    NECK_PITCH_60  = 60,
} neck_pitch_t;
#define NECK_1_MIN 75
#define NECK_1_MID 120
#define NECK_1_MAX 140

#define NECK_2_MIN 75
#define NECK_2_MID 120
#define NECK_2_MAX 140


#define NECK_3_MIN 75
#define NECK_3_MID 120
#define NECK_3_MAX 140

#define NECK_R_MIN 50    //0
#define NECK_R_MID 150
#define NECK_R_MAX 250   

#define NeckNod_DOWN 0
#define NeckNod_UP 1
#define NeckNod_LEFT 2
#define NeckNod_RIGHT 3

void NeckInt();
void NeckNod();
void NeckUpDown();
void NeckShow();
void TestNod();
void Neckinit();
uint16_t neck_angle_to_ccr(int16_t angle);
void Neck_R_SetTargetAngle(int16_t angle);
void Neck_R_Update(void);
void Neck_Pitch_Update(void);
void Neck_Pitch_SetTargetAngle(int16_t angle);
#endif
/* USER CODE END */
