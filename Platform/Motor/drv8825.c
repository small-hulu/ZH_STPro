/*
 * drv8825.c
 *
 *  Created on: Nov 26, 2025
 *      Author: ZEHO
 */

#include "drv8825.h"
#include "stm32f4xx_hal.h"

/**
 * @brief  初始化DRV8825（引脚配置+细分设置）
 * @param  microstep：初始细分模式
 */
void DRV8825_Init(DRV8825_MicroStepTypeDef microstep) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    // 配置控制引脚为推挽输出
    GPIO_InitStruct.Pin = DRV8825_STEP_Pin | DRV8825_DIR_Pin |
                          DRV8825_EN_Pin | DRV8825_MS0_Pin |
                          DRV8825_MS1_Pin | DRV8825_MS2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; 
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		HAL_GPIO_WritePin(DRV8825_EN_GPIO_Port, DRV8825_EN_Pin, GPIO_PIN_SET);
    DISABLE_8825;
    STEP_LOW;

    // 设置细分模式
    DRV8825_SetMicroStep(microstep);

    // 使能驱动
    ENABLE_8825;
}

/**
 * @brief  设置细分模式（通过MS1、MS2、MS3引脚）
 * @param  microstep：目标细分模式
 */
void DRV8825_SetMicroStep(DRV8825_MicroStepTypeDef microstep) {
    // 默认为低电平（0）
    HAL_GPIO_WritePin(DRV8825_MS0_GPIO_Port, DRV8825_MS0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV8825_MS1_GPIO_Port, DRV8825_MS1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV8825_MS2_GPIO_Port, DRV8825_MS2_Pin, GPIO_PIN_RESET);

    switch(microstep) {
        case MICROSTEP_1:   // 1/1步：MS0=0, MS1=0, MS2=0
            break;
        case MICROSTEP_2:   // 1/2步：MS0=1, MS1=0, MS2=0
            HAL_GPIO_WritePin(DRV8825_MS0_GPIO_Port, DRV8825_MS0_Pin, GPIO_PIN_SET);
            break;
        case MICROSTEP_4:   // 1/4步：MS0=0, MS1=1, MS2=0
            HAL_GPIO_WritePin(DRV8825_MS1_GPIO_Port, DRV8825_MS1_Pin, GPIO_PIN_SET);
            break;
        case MICROSTEP_8:   // 1/8步：MS0=1, MS1=1, MS2=0
            HAL_GPIO_WritePin(DRV8825_MS0_GPIO_Port, DRV8825_MS0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(DRV8825_MS1_GPIO_Port, DRV8825_MS1_Pin, GPIO_PIN_SET);
            break;
        case MICROSTEP_16:  // 1/16步：MS0=1, MS1=1, MS2=1
            HAL_GPIO_WritePin(DRV8825_MS0_GPIO_Port, DRV8825_MS0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(DRV8825_MS1_GPIO_Port, DRV8825_MS1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(DRV8825_MS2_GPIO_Port, DRV8825_MS2_Pin, GPIO_PIN_SET);
            break;
        case MICROSTEP_32:  // 1/32步：MS0=0, MS1=0, MS2=1
            HAL_GPIO_WritePin(DRV8825_MS2_GPIO_Port, DRV8825_MS2_Pin, GPIO_PIN_SET);
            break;
        default:
            break;
    }
}

/**
 * @brief  控制舵机转动指定步数
 * @param  dir：方向（0=顺时针，1=逆时针）
 * @param  steps：步数（根据细分模式，1圈步数=基础步数×细分倍数，如基础200步/圈，16细分则3200步/圈）
 * @param  step_delay_us：每步间隔（控制转速，最小延迟受芯片和电源限制）
 */
void DRV8825_Rotate(uint8_t dir, uint32_t steps, uint16_t step_delay_us) {
    // 设置方向
		//ENABLE;
		if (dir == 0) DIR_CW;
    else DIR_CCW;

    // 输出步进脉冲（上升沿触发）
    for (uint32_t i = 0; i < steps; i++) {
        STEP_HIGH;
        HAL_Delay(step_delay_us / 1000);  // 毫秒级延迟（需微秒级可替换为定时器）
        STEP_LOW;
        HAL_Delay(step_delay_us / 1000);
    }
		//DISABLE;
}

/**
 * @brief  停止舵机（保持当前位置，若需释放可禁用驱动）
 */
void DRV8825_Stop(void) {
    // 无需额外操作，停止输出脉冲即可保持位置
    // 若需释放扭矩，可调用DISABLE：DISABLE();
}

