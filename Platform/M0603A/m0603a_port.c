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
    if (m0603a_lib_lock(handle))
    {
        HAL_UART_Transmit(handle->info->uart_handle, txbuf, num, 100);
        HAL_UART_Receive(handle->info->uart_handle, rxbuf, num, 100);
        m0603a_lib_unlock(handle);
    }

}
