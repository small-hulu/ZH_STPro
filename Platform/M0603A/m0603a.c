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
 * @brief   m0603a lib handle init
 * @param
 * @note
 * @retval
 */
void m0603a_lib_handle_init(m0603a_lib_handle* handle, m0603a_info* info, uint32_t dev_num,
                             int (*lock)(uint8_t)) {
    handle->info    = info;
    handle->dev_num = dev_num;
    handle->lock    = lock;
}
				

/**
 * @brief   tmc51x0 lib lock
 * @note
 */
int m0603a_lib_lock(m0603a_lib_handle* handle) {
    if (handle->lock == 0) {
        return 1;
    }

    return handle->lock(1);
}
/**
 * @brief   tmc51x0 lib unlock
 * @note
 */
int m0603a_lib_unlock(m0603a_lib_handle* handle) {
    if (handle->lock == 0) {
        return 1;
    }

    return handle->lock(0);
}

/* ================= CRC8 ================= */
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

/* ================= 基础控制 ================= */
uint8_t rx_new[10] = {0};
void Motor_Enable_new(m0603a_lib_handle* handle, uint8_t id)
{
		//uint8_t rx[10] = {0};
    uint8_t f[10] = {id, MOTOR_CMD_MODE, MOTOR_MODE_ENABLE};
		f[9] = crc8_maxim(f, 9);
    m0603a_lib_uart_send_rev(handle, f, rx_new, 10);
}
