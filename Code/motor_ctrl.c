/***************************************************************************
 * Copyright  2025
 * All right reserved. See COPYRIGHT for detailed Information.
 *
 * @file        motor_ctrl.c
 *
 * @author     
 * @brief      
 *
 * @Email      
 *
 * @date       2025/12/22
 * @version    0.0.1
 ***************************************************************************/
#include "motor_ctrl.h"
#include "cmsis_os.h"
/**
 * @brief  moto L info
 */
 m0603a_info motor_l_info = {.uart_handle = &huart2};
 
 
 /**
 * @brief  moto L handle
 */
 m0603a_lib_handle motor_l_handle;
 
 
 
 /**
 * @brief  moto L mutex attr
 */
 const osMutexAttr_t moto_l_mutex_attr = {
    .name = "moto_l_mutex",
    .attr_bits = osMutexPrioInherit | osMutexRecursive | osMutexRobust
};

/**
 * @brief  moto L mutex
 */
osMutexId_t moto_l_mutex = NULL;
 
 /**
 * @brief  moto L lock method
 */
static inline int moto_l_lock(uint8_t lock) {
    if (lock) {
        return osMutexAcquire(moto_l_mutex, osWaitForever);
    } else {
        return osMutexRelease(moto_l_mutex);
    }
}



void moto_l_init()
{
		moto_l_mutex = osMutexNew(&moto_l_mutex_attr);
    m0603a_lib_handle_init(&motor_l_handle, &motor_l_info, 1, moto_l_lock);
		Motor_Enable_new(&motor_l_handle,1);
}