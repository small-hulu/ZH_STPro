/*
 * StepMotor.h
 *
 *  Created on: Nov 26, 2025
 *      Author: ZEHO
 */

#ifndef INC_STEPMOTOR_H_
#define INC_STEPMOTOR_H_

#include "stm32f4xx_hal.h"

#define Left  0
#define Right 1
#define STEPPER_FORWARD  0
#define STEPPER_BACKWARD 1
/*#define STEPPER_PORT     GPIOE
#define IN1_PIN          GPIO_PIN_0  //A+
#define IN2_PIN          GPIO_PIN_1  //B+
#define IN3_PIN          GPIO_PIN_2  //A-
#define IN4_PIN          GPIO_PIN_3	 //B-
*/

#define STEPPER_PORT_L    GPIOG
#define IN1_PIN_L         GPIO_PIN_4  // ???? A+
#define IN2_PIN_L         GPIO_PIN_6  // ???? B+
#define IN3_PIN_L         GPIO_PIN_5  // ???? A-
#define IN4_PIN_L         GPIO_PIN_7  // ???? B-

#define STEPPER_PORT_R    GPIOD
#define IN1_PIN_R         GPIO_PIN_8  // ???? A+
#define IN2_PIN_R         GPIO_PIN_10  // ???? B+
#define IN3_PIN_R         GPIO_PIN_9  // ???? A-
#define IN4_PIN_R         GPIO_PIN_11  // ???? B-

#define STEPS_PER_REV   64
#define GEAR_RATIO      8
#define TOTAL_STEPS_PER_REV (STEPS_PER_REV * 8 * GEAR_RATIO)


void Stepper_Init(void);
void Stepper_SetSpeed(uint16_t rpm);
void Stepper_Step(uint8_t side, uint8_t direction);
void Stepper_RotateAngle1(float angle, uint8_t direction);
void Stepper_RotateAngle2(float angle, uint8_t direction);
void Stepper_RotateCircle(uint8_t side, float circles, uint8_t direction);
void Stepper_Stop(uint8_t side);

#endif /* INC_STEPMOTOR_H_ */
