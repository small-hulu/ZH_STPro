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
#ifndef __M0603A_H
#define __M0603A_H

#include "usart.h"

#include "user_main.h"
struct m0603a_info;
typedef struct m0603a_info m0603a_info;
typedef struct {
    int32_t mileage;
    int16_t cur_pos;
		int16_t rpm_raw;
		int16_t rpm;
    int status;
} MotorStatus_t;

typedef struct {
    m0603a_info* info;
    uint8_t      dev_num;
    int (*lock)(uint8_t lock_state);  // lock_state=0: unlock;1=lock
		MotorStatus_t status;
}m0603a_lib_handle;


extern m0603a_lib_handle motor_r_handle;
extern m0603a_lib_handle motor_l_handle;

/* CMD */
#define MOTOR_CMD_MODE        0xA0
#define MOTOR_CMD_RUN         0x64
#define MOTOR_CMD_READ        0xA1   
#define MOTOR_CMD_MIL         0x74   
#define MOTOR_CMD_CUR_MODE    0x75
#define MOTOR_CMD_VERSION     0xFD

/* MODE */
#define MOTOR_MODE_OPEN       0x00
#define MOTOR_MODE_CURR       0x01   // current mode
#define MOTOR_MODE_SPEED      0x02   // speed mode
#define MOTOR_MODE_POS        0x03   // pos mode
#define MOTOR_MODE_ENABLE     0x08
#define MOTOR_MODE_DISABLE    0x09

int m0603a_lib_lock(m0603a_lib_handle* handle);
int m0603a_lib_unlock(m0603a_lib_handle* handle);
void m0603a_lib_handle_init(m0603a_lib_handle* handle, m0603a_info* info, uint8_t dev_num,int (*lock)(uint8_t));
void m0603a_lib_set_mode(m0603a_lib_handle* handle,uint8_t mode);
void m0603a_lib_set_motor_en(m0603a_lib_handle* handle);
void m0603a_lib_set_speed(m0603a_lib_handle* handle, int16_t rpm, uint8_t acc);
int16_t m0603a_lib_set_get_speed(m0603a_lib_handle* handle, int16_t rpm, uint8_t acc);
int32_t m0603a_lib_get_mileage_count(m0603a_lib_handle* handle);
int16_t m0603a_lib_get_cur_pos(m0603a_lib_handle* handle);

//要改
int m0603a_lib_get_status(m0603a_lib_handle* handle);

#endif
