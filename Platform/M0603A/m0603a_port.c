/***************************************************************************
 * Copyright  2026 xushaowei
 * All right reserved. See COPYRIGHT for detailed Information.
 *
 * @file       m0603a.c
 *
 * @author     xushaowei
 * @brief      m0603a control
 *
 * @Email      
 *
 * @date       2026/1/27
 * @version    0.0.1
 ***************************************************************************/
#include "m0603a_port.h"
/**
 * @brief   m0603a uart wirte data
 * @param
 * @note
 * @retval
 */
void m0603a_lib_uart_send(m0603a_lib_handle* handle, uint8_t* buf, uint32_t num)
{
    if (m0603a_lib_lock(handle))
    {
        HAL_UART_Transmit(handle->info->uart_handle, buf, num, 100);
        m0603a_lib_unlock(handle);
    }
}

/**
 * @brief   m0603a uart read data
 * @param
 * @note
 * @retval
 */
void m0603a_lib_uart_rev(m0603a_lib_handle* handle, uint8_t* buf, uint32_t num)
{
    if (m0603a_lib_lock(handle))
    {
        HAL_UART_Receive(handle->info->uart_handle, buf, num, 100);
        m0603a_lib_unlock(handle);
    }
}

/**
 * @brief   m0603a uart sen and reveice data
 * @param
 * @note   __weak function ,need user redefine
 * @retval
 */
void m0603a_lib_uart_send_rev(m0603a_lib_handle* handle,
                                     uint8_t* txbuf,
                                     uint8_t* rxbuf,
                                     uint32_t num)
{
	  uint8_t  tmp;
		uint32_t lock_state;
		while (!HAL_UART_Receive(handle->info->uart_handle, &tmp, 1, 1));
		lock_state = osKernelLock();
    if (m0603a_lib_lock(handle))
    {
        HAL_UART_Transmit(handle->info->uart_handle, txbuf, num, 5);
        HAL_UART_Receive(handle->info->uart_handle, rxbuf, num, 10);
        m0603a_lib_unlock(handle);
				osKernelRestoreLock(lock_state);
    }

}
