/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "user_main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for fun_ctrl_task */
osThreadId_t fun_ctrl_taskHandle;
const osThreadAttr_t fun_ctrl_task_attributes = {
  .name = "fun_ctrl_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow3,
};
/* Definitions for status_task */
osThreadId_t status_taskHandle;
const osThreadAttr_t status_task_attributes = {
  .name = "status_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for neckTask */
osThreadId_t neckTaskHandle;
const osThreadAttr_t neckTask_attributes = {
  .name = "neckTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
typedef struct {
    int16_t angle; 
		int16_t pitch_angle;  
} neck_cmd_t;

QueueHandle_t neckQueue;
QueueHandle_t uartQueue;
fsm_lib_return main_ctrl_fsm(fsm_lib_ctrl_handle *handle);
fsm_lib_ctrl_handle main_ctrl_fsm_handle;

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void fun_ctrl_Task(void *argument);
void Status_Task(void *argument);
void NeckTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

	
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	
	uartQueue = xQueueCreate(4,sizeof(uart_frame_t));
	configASSERT(uartQueue != NULL);
	neckQueue = xQueueCreate(8, sizeof(neck_cmd_t)); 
	configASSERT(neckQueue != NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of fun_ctrl_task */
  fun_ctrl_taskHandle = osThreadNew(fun_ctrl_Task, NULL, &fun_ctrl_task_attributes);

  /* creation of status_task */
  status_taskHandle = osThreadNew(Status_Task, NULL, &status_task_attributes);

  /* creation of neckTask */
  neckTaskHandle = osThreadNew(NeckTask, NULL, &neckTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
serial_frame_t pkt;
cmd_vel_t cmd;
neck_cmd_t neck_cmd;
uint8_t  cur_cmd;
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	//printf("StartDefaultTask\r\n");
  uint16_t rx_len;
	uart_frame_t frame;
	uint8_t tx_buf[32];
	uint16_t tx_len;

	for (;;)
	{	
			if (xQueueReceive(uartQueue,&frame,portMAX_DELAY) == pdPASS)
      {
					//Unpacking
					//printf("%s", frame.data);
					serial_frame_ret_t ret =
                serial_frame_parse((uint8_t *)frame.data, frame.len, &pkt);
						cur_cmd = pkt.cmd;
            if (ret == SERIAL_FRAME_OK)
            {
                switch (pkt.cmd)
                {
                case 0x01:
								{
												cmd.linear_x_mm_s =
									(int16_t)((pkt.data[0] << 8) | pkt.data[1]);
												cmd.angular_z_mrad =
									(int16_t)((pkt.data[2] << 8) | pkt.data[3]);

					
									  Motor_SetSpeed_FromCmd(&cmd, 100);
									  //send cmd and v 
										uint8_t ack_data[] = {0x00};
										tx_len = serial_frame_build(0x81, ack_data, 1,tx_buf, sizeof(tx_buf));
										HAL_UART_Transmit(&huart6, tx_buf, tx_len, 100);
                    break;
								}
                
                case 0x02:
								{
									  ArmFrontBack();
										uint8_t ack_data[] = {0x00};
										tx_len = serial_frame_build(0x82, ack_data, 1,tx_buf, sizeof(tx_buf));
										HAL_UART_Transmit(&huart6, tx_buf, tx_len, 100);
                    break;
								}   
								case 0x03:
								{
										//ack
										uint8_t ack_data[] = {0x00};
										tx_len = serial_frame_build(0x83, ack_data, 1,tx_buf, sizeof(tx_buf));
										HAL_UART_Transmit(&huart6, tx_buf, tx_len, 100);
                    break;
								}
                case 0x04:
								{
										//ack
										uint8_t ack_data[] = {0x00};
										tx_len = serial_frame_build(0x84, ack_data, 1,tx_buf, sizeof(tx_buf));
										HAL_UART_Transmit(&huart6, tx_buf, tx_len, 100);
                    break;
								}
								case 0x05:
								{
										//ack
										neck_cmd.angle       = (int16_t)((pkt.data[0] << 8) | pkt.data[1]);
										neck_cmd.pitch_angle = (int16_t)((pkt.data[2] << 8) | pkt.data[3]);
  
										xQueueSendToBack(neckQueue, &neck_cmd, 0);
  
										uint8_t ack_data[] = {0x00};
										tx_len = serial_frame_build(0x85, ack_data, 1,tx_buf, sizeof(tx_buf));
										HAL_UART_Transmit(&huart6, tx_buf, tx_len, 100);
                    break;
								}
                default:
                    break;
                }
            }
            else
            {
            }
      }
			osDelay(5);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_fun_ctrl_Task */
/**
* @brief Function implementing the fun_ctrl_task thread.
* @param argument: Not used
* @retval None
	
*/
/* USER CODE END Header_fun_ctrl_Task */
void fun_ctrl_Task(void *argument)
{
  /* USER CODE BEGIN fun_ctrl_Task */
	
	//neck_init
		Neckinit();
	
	fsm_lib_start(&main_ctrl_fsm_handle);
  /* Infinite loop */
	int16_t start_angle = 70;
  int16_t end_angle   = 140;
	int16_t pitch_start = 0;
  int16_t pitch_end   = 60;
  neck_cmd_t cmd;
	Stop();
	//GoAhead();
	ArmUpDown();
  for(;;)
  {
         // ??? -> ??
//        cmd.angle       = end_angle;
//        cmd.pitch_angle = pitch_end;
//        xQueueSendToBack(neckQueue, &cmd, 0);
//        osDelay(2000); // ???????

//        // ??? -> ??
//        cmd.angle       = start_angle;
//        cmd.pitch_angle = pitch_start;
//        xQueueSendToBack(neckQueue, &cmd, 0);
        osDelay(1000);
  }
  /* USER CODE END fun_ctrl_Task */
}

/* USER CODE BEGIN Header_Status_Task */
/**
* @brief Function implementing the status_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Status_Task */
void Status_Task(void *argument)
{
  /* USER CODE BEGIN Status_Task */
  /* Infinite loop */
  for(;;)
  {
		//touch senior  todo
		HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_10);
		osDelay(1000);
  }
  /* USER CODE END Status_Task */
}

/* USER CODE BEGIN Header_NeckTask */
/**
* @brief Function implementing the neckTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NeckTask */
void NeckTask(void *argument)
{
  /* USER CODE BEGIN NeckTask */
  /* Infinite loop */
	TickType_t last = xTaskGetTickCount();
	neck_cmd_t cmd;
  for(;;)
  {
      if (xQueueReceive(neckQueue, &cmd, portMAX_DELAY) == pdTRUE)
      {
          Neck_R_SetTargetAngle(cmd.angle);
					Neck_Pitch_SetTargetAngle(cmd.pitch_angle);
          
          last = xTaskGetTickCount();
          while (neck_r_cur_angle != neck_r_target_angle || neck_pitch_cur != neck_pitch_target)
					{
							Neck_Pitch_Update(); 
						
							if (xQueueReceive(neckQueue, &cmd, 0) == pdTRUE)
							{
									Neck_R_SetTargetAngle(cmd.angle);
									Neck_Pitch_SetTargetAngle(cmd.pitch_angle);
							}
							vTaskDelay(pdMS_TO_TICKS(25));
					}
      }
  }
  /* USER CODE END NeckTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
 * @brief   fun_ctrl fsm
 */
fsm_lib_return main_ctrl_fsm(fsm_lib_ctrl_handle *handle) {
    enum {
        idle = 0,
        start,
        end,
        error,
        seft_tset,
        running,
        exit,
        func_error,
    };
    fsm_lib_check_emergency(handle);

    switch (handle->state) {
        fsm_lib_state(idle) {
						printf("main_ctrl_fsm idle\r\n");
            fsm_lib_release(handle, fsm_rt_idle);
        }
        break;

        fsm_lib_state(start) {
					  printf("main_ctrl_fsm start\r\n");
            fsm_lib_update_to(handle, seft_tset);
        }
        break;

        fsm_lib_state(end) {
            fsm_lib_transfer_to(handle, idle, fsm_rt_cpl);
        }
        break;

        fsm_lib_state(error) {}
        break;
        fsm_lib_state(seft_tset) {
            printf("main_ctrl_fsm seft_tset\r\n");
					  fsm_lib_update_to(handle, running);
        }
        break;
        fsm_lib_state(running) {
            printf("main_ctrl_fsm running\r\n");
						fsm_lib_release(handle, fsm_rt_running);
        }
        break;
        fsm_lib_state(exit) {

        }
        break;
        fsm_lib_state(func_error) {
            
        }
        break;
    }

    return fsm_rt_running;
}
/* USER CODE END Application */

