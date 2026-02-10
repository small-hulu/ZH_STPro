/***************************************************************************
 * Copyright  2026 
 * All right reserved. See COPYRIGHT for detailed Information.
 *
 * @file        motor_ctrl.h
 *
 * @author     xushaowei
 * @brief      
 *
 * @Email     
 *
 * @date       
 * @version    0.0.1
 ***************************************************************************/
#ifndef __MOTOR_CTRL_H__
#define __MOTOR_CTRL_H__
#include "user_main.h"





void moto_l_init();
void moto_r_init();
void motor_set_get_speed_from_cmd(const cmd_vel_t *cmd, uint8_t acc, float *v_left_out, float *v_right_out);
#endif
