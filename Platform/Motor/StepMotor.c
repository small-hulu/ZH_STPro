/*
 * StepMotor.c
 *
 *  Created on: Nov 26, 2025
 *      Author: ZEHO
 */

#include "stepmotor.h"
#include "stm32f4xx_hal.h"

/**
 * @brief
 * @param
 */
uint8_t currentStep = 0;
uint32_t stepDelay = 15000;//us
const uint8_t stepSequence[8] = {
		//IN1=A+,IN2=B+,IN3=A-,IN4=B-
		0x01,//1,0,0,0
		0x03,//1,1,0,0
		0x02,//0,1,0,0
		0x06,//0,1,1,0
		0x04,//0,0,1,0
		0x0C,//0,0,1,1
		0x08,//0,0,0,1
		0x09,//1,0,0,1

		/*0x01,
		0x02,
		0x04,
		0x08,*/
};

void Stepper_Init(){
	Stepper_Stop(Left);
	Stepper_Stop(Right);
}

void Stepper_SetSpeed(uint16_t rpm) {
    if (rpm == 0) return;
    stepDelay = (60 * 1000000) / (rpm * TOTAL_STEPS_PER_REV);
}

static void setOutput(uint8_t side,uint8_t value) {
	if(side == Left){
		HAL_GPIO_WritePin(STEPPER_PORT_L, IN1_PIN_L, (value & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEPPER_PORT_L, IN2_PIN_L, (value & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEPPER_PORT_L, IN3_PIN_L, (value & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEPPER_PORT_L, IN4_PIN_L, (value & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin(STEPPER_PORT_R, IN1_PIN_R, (value & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEPPER_PORT_R, IN2_PIN_R, (value & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEPPER_PORT_R, IN3_PIN_R, (value & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEPPER_PORT_R, IN4_PIN_R, (value & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}

}

void Stepper_Step(uint8_t side,uint8_t direction) {
	if (direction == STEPPER_FORWARD)
			{
			currentStep = (currentStep + 1) % 8;
				//currentStep++;
		   // if (currentStep >= 8) currentStep = 0;
		} else
			{
			if (currentStep == 0) currentStep = 7;
				else currentStep--;
		}

	setOutput(side,~stepSequence[currentStep]);  //UNL2003 ~
    HAL_Delay(stepDelay/2950);
 /*   if (stepDelay % 1000 > 0) {
        delay_us(stepDelay % 1000);
    }*/
}

void Stepper_RotateAngle1(float angle, uint8_t direction) {
    uint32_t steps = (uint32_t)((angle / 360.0f) * TOTAL_STEPS_PER_REV)/2;
    for (uint32_t i = 0; i < steps; i++) {
        Stepper_Step(Left,direction);
        Stepper_Step(Right,direction);
    }
    Stepper_Stop(Left);
    Stepper_Stop(Right);
}
void Stepper_RotateAngle2(float angle, uint8_t direction) {
    uint32_t steps = (uint32_t)((angle / 360.0f) * TOTAL_STEPS_PER_REV)/2;

    for (uint32_t i = 0; i < steps; i++) {
        //Stepper_Step(Left,direction);
        Stepper_Step(Right,direction);
    }
    //Stepper_Stop(Left);
    Stepper_Stop(Right);
}

void Stepper_RotateCircle(uint8_t side, float circles, uint8_t direction) {
    uint32_t steps = (uint32_t)(circles * TOTAL_STEPS_PER_REV);

    for (uint32_t i = 0; i < steps; i++) {
        Stepper_Step(side,direction);
    }
}


void Stepper_Stop(uint8_t side) {
    setOutput(side,0x00);
}

