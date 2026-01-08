/***************************************************************************
 * Copyright  2025 
 * All right reserved. See COPYRIGHT for detailed Information.
 *
 * @file        user_main.h
 *
 * @author     
 * @brief      
 *
 * @Email     
 *
 * @date       
 * @version    0.0.1
 ***************************************************************************/
#ifndef __USER_MAIN_H__
#define __USER_MAIN_H__

// firmware version
#define VER_MAJOR         1
#define VER_MINOR         1
#define VER_PATCH         1
#define FIRMWARE_TYPE_APP 1

// bsp include
#include "cmsis_os2.h"
#include <stdio.h>

#include "main.h"

//Middlewares
#include "FreeRTOS.h"
#include "queue.h"
#include "freertos.h"

// platform
#include "fsm_lib.h"
#include "act.h"
#include "motor.h"
#include "tim.h"
#include "StepMotor.h"
#include "serial_frame.h"
#include "Neck.h"
// app

void bsp_init();
void user_main();

#endif
