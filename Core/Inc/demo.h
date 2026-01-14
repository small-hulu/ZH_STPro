/**
 ****************************************************************************************************
 * @file        demo.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS6DSV模块测试实验
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 探索者 F407开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#ifndef __DEMO_H
#define __DEMO_H

#include "../Platform/IMU/atk_ms6dsv.h"
#include "./usart.h"
#include "../Platform/IMU/delay.h"
void demo_run(void);

//float* Get_Acc(void);
//float* Get_Ang(void);
//float Get_Acc_z(void);
//uint8_t ret;
//lsm6dsv16x_filt_settling_mask_t filt_settling_mask;
//lsm6dsv16x_data_ready_t drdy;
//int16_t data_raw_acceleration[3];
//int16_t data_raw_angular_rate[3];
//int16_t data_raw_temperature;
//    
//uint8_t i2ctimes;
//		
//float acceleration_mg[3];
//float angular_rate_mdps[3];

#endif
