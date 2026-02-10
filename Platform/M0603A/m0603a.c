/***************************************************************************
 * Copyright  2026 xushaowei
 * All right reserved. See COPYRIGHT for detailed Information.
 *
 * @file        m0603a.c
 *
 * @author     xushaowei
 * @brief      m0603a control
 *
 * @Email      
 *
 * @date       2026/1/27
 * @version    0.0.1
 ***************************************************************************/
#include "m0603a.h"
#define __weak __attribute((weak))
#define __IO   volatile

/**
 * @brief   m0603a uart sen and reveice data
 * @param
 * @note   __weak function ,need user redefine
 * @retval
 */
__weak void m0603a_lib_uart_send_rev(m0603a_lib_handle* handle, uint8_t* txbuf, uint8_t* rxbuf,
                                     uint32_t num) {}
/**
 * @brief   m0603a uart send data
 * @param
 * @note   __weak function ,need user redefine
 * @retval
 */
__weak void m0603a_lib_uart_send(m0603a_lib_handle* handle, uint8_t* buf, uint32_t num) {}

/**
 * @brief   m0603a uart reveice data
 * @param
 * @note   __weak function ,need user redefine
 * @retval
 */
__weak void m0603a_lib_uart_rev(m0603a_lib_handle* handle, uint8_t* buf, uint32_t num) {}

				

/**
 * @brief   m0603a lib lock
 * @note
 */
int m0603a_lib_lock(m0603a_lib_handle* handle) {
    if (handle->lock == 0) {
        return 1;
    }

    return handle->lock(1);
}
/**
 * @brief   m0603a lib unlock
 * @note
 */
int m0603a_lib_unlock(m0603a_lib_handle* handle) {
    if (handle->lock == 0) {
        return 1;
    }

    return handle->lock(0);
}
/**
 * @brief   m0603a lib handle init
 * @param
 * @note
 * @retval
 */
void m0603a_lib_handle_init(m0603a_lib_handle* handle, m0603a_info* info, uint8_t dev_num,
                             int (*lock)(uint8_t)) {
    handle->info    = info;
    handle->dev_num = dev_num;
    handle->lock    = lock;
}
														 
static uint8_t crc8_maxim(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 1) ? ((crc >> 1) ^ 0x8C) : (crc >> 1);
    }
    return crc;
}		

/**
 * @brief   m0603a lib set mode
 * @param
 * @note
 * @retval
 */
void m0603a_lib_set_mode(m0603a_lib_handle* handle,uint8_t mode) {
    uint8_t rx[10];
    uint8_t f[10] = {handle->dev_num, MOTOR_CMD_MODE, mode};
		f[9] = crc8_maxim(f, 9);
    m0603a_lib_uart_send_rev(handle, f, rx, 10);
}


/**
 * @brief   m0603a lib set mode
 * @param
 * @note
 * @retval
 */
void m0603a_lib_set_motor_en(m0603a_lib_handle* handle) {

		m0603a_lib_set_mode(handle,MOTOR_MODE_ENABLE);
}

/**
 * @brief   m0603a_lib_set_speed
 * @note
 */
void m0603a_lib_set_speed(m0603a_lib_handle* handle, int16_t rpm, uint8_t acc)
{
		uint8_t f[10] = {handle->dev_num, MOTOR_CMD_RUN};
		uint8_t rx[10];
    f[2] = rpm >> 8;
    f[3] = rpm & 0xFF;

    f[6] = acc;
		f[9] = crc8_maxim(f, 9);
		
    m0603a_lib_uart_send_rev(handle, f, rx, 10);
}
/**
 * @brief   m0603a_lib_set_get_speed
 * @note
 */
int16_t m0603a_lib_set_get_speed(m0603a_lib_handle* handle, int16_t rpm, uint8_t acc)
{
    uint8_t f[10] = {handle->dev_num, MOTOR_CMD_RUN};
    int16_t speed_raw = 0;
    uint8_t rx[10];

    int16_t rpm_raw = rpm;   

    f[2] = rpm_raw >> 8;
    f[3] = rpm_raw & 0xFF;
    f[6] = acc;
    f[9] = crc8_maxim(f, 9);

    m0603a_lib_uart_send_rev(handle, f, rx, 10);

    if (crc8_maxim(rx, 9) != rx[9]) {
        return -1;   
    }

    if (rx[0] != handle->dev_num || rx[1] != 0x65) {
        return -2;  
    }

    speed_raw = (int16_t)((rx[2] << 8) | rx[3]);  

    handle->status.rpm_raw = speed_raw;  

    return speed_raw;  
}


/**
 * @brief   m0603a_lib_get_mileage_count
 * @note
 */
int32_t m0603a_lib_get_mileage_count(m0603a_lib_handle* handle)
{
    uint8_t f[10] = {0};
		uint8_t rx[10];
		static int32_t mileage = 0;
    f[0] = handle->dev_num;
    f[1] = MOTOR_CMD_MIL;

    f[9] = crc8_maxim(f, 9);

    m0603a_lib_uart_send_rev(handle, f, rx, 10);
		uint8_t crc_ret = crc8_maxim(rx, 9);
    if (crc_ret != rx[9]) {
        return mileage;
    }

    if (rx[0] != handle->dev_num || rx[1] != 0x75) {
        return mileage;
    }

    mileage =
        ((int32_t)rx[2] << 24) |
        ((int32_t)rx[3] << 16) |
        ((int32_t)rx[4] << 8)  |
        ((int32_t)rx[5]);
		handle->status.mileage = mileage;
    return mileage;
		
}
/**
 * @brief   m0603a_lib_get_cur_pos
 * @note
 */
int16_t m0603a_lib_get_cur_pos(m0603a_lib_handle* handle)
{
    uint8_t f[10] = {0};
		uint8_t rx[10];
		int16_t pos = 0;
    f[0] = handle->dev_num;
    f[1] = MOTOR_CMD_MIL;

    f[9] = crc8_maxim(f, 9);

    m0603a_lib_uart_send_rev(handle, f, rx, 10);
    if (crc8_maxim(rx, 9) != rx[9]) {
        return 0;
    }

    if (rx[0] != handle->dev_num || rx[1] != 0x75) {
        return 0;
    }

    pos = (int16_t)(((uint8_t)rx[6] << 8) | rx[7]);
		handle->status.cur_pos = pos;
    return pos;
		
}

int m0603a_lib_get_status(m0603a_lib_handle* handle)
{
    if (!handle) return -1;

    int ret = 0;

		uint8_t f[10] = {0};
		uint8_t rx[10] = {0};

		f[0] = handle->dev_num;
		f[1] = MOTOR_CMD_MIL;
		f[9] = crc8_maxim(f, 9);

		m0603a_lib_uart_send_rev(handle, f, rx, 10);

		if (crc8_maxim(rx, 9) != rx[9])
		{
				ret = 1; 
		}
		else if (rx[0] != handle->dev_num || rx[1] != 0x75)
		{
				ret = 2;
		}
		else
		{
				handle->status.mileage =
						((int32_t)rx[2] << 24) |
						((int32_t)rx[3] << 16) |
						((int32_t)rx[4] << 8) |
						((int32_t)rx[5]);

				handle->status.cur_pos = (int16_t)(((uint8_t)rx[6] << 8) | rx[7]);
		}


    handle->status.status = ret;
    return ret;
}
