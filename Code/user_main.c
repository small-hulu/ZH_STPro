/***************************************************************************
 * Copyright  2025
 * All right reserved. See COPYRIGHT for detailed Information.
 *
 * @file        user_main.c
 *
 * @author     
 * @brief      
 *
 * @Email      
 *
 * @date       2025/12/22
 * @version    0.0.1
 ***************************************************************************/
#include "user_main.h"

/**
 * @brief   user main
 * @param
 * @note
 * @retval
 */
void user_main() {
    //osKernelInitialize();
    bsp_init();

		
    // initialize rtos kernel
    // task init
    // os start, never return if success
    //osKernelStart();

    // kernel start failed
    while (1) {
    }
}

/**
 * @brief   bsp init
 * @param
 * @note
 * @retval
 */
void bsp_init() {
    // set systick IRQ priority
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0U);

}


uint32_t swdt_get_tick() {
    return HAL_GetTick();
}
