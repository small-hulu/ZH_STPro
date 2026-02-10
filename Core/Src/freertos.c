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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "user_main.h"
#include "demo.h"
#include "../Platform/IMU/lsm6dsv16x_reg.h"
#include "../Platform/IMU/atk_ms6dsv.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	uint8_t ret;
	int16_t data_raw_acceleration[3];
	int16_t data_raw_angular_rate[3];
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
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for neckTask */
osThreadId_t neckTaskHandle;
const osThreadAttr_t neckTask_attributes = {
  .name = "neckTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartTxTask */
osThreadId_t uartTxTaskHandle;
const osThreadAttr_t uartTxTask_attributes = {
  .name = "uartTxTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for wheel_status_ta */
osThreadId_t wheel_status_taHandle;
const osThreadAttr_t wheel_status_ta_attributes = {
  .name = "wheel_status_ta",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
typedef struct {
    int16_t angle; 
		int16_t pitch_angle;  
} neck_cmd_t;

QueueHandle_t neckQueue;
QueueHandle_t uartQueue;
QueueHandle_t uartTxQueue;
QueueHandle_t motorCmdQueue;

osMutexId_t uart_mutex;
fsm_lib_return main_ctrl_fsm(fsm_lib_ctrl_handle *handle);
fsm_lib_ctrl_handle main_ctrl_fsm_handle;
int32_t mileage_r = 0;
int16_t cur_pos_r = 0;
int32_t mileage_l = 0;
int16_t cur_pos_l = 0;
uint8_t send_data[16];
uart_tx_frame_t tx;
uint32_t ts;
volatile uint32_t imu_time_base = 0;

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void fun_ctrl_Task(void *argument);
void Status_Task(void *argument);
void NeckTask(void *argument);
void UartTxTask(void *argument);
void Wheel_Status_Task(void *argument);

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
	
	uartQueue = xQueueCreate(8,sizeof(uart_frame_t));
	configASSERT(uartQueue != NULL);
	uartTxQueue = xQueueCreate(8, sizeof(uart_tx_frame_t));
	configASSERT(uartTxQueue);
	neckQueue = xQueueCreate(8, sizeof(neck_cmd_t)); 
	configASSERT(neckQueue != NULL);
	motorCmdQueue = xQueueCreate(4, sizeof(cmd_vel_t));
	configASSERT(motorCmdQueue != NULL);
	
	uart_mutex = osMutexNew(NULL);
  configASSERT(uart_mutex != NULL);
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

  /* creation of uartTxTask */
  uartTxTaskHandle = osThreadNew(UartTxTask, NULL, &uartTxTask_attributes);

  /* creation of wheel_status_ta */
  wheel_status_taHandle = osThreadNew(Wheel_Status_Task, NULL, &wheel_status_ta_attributes);

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
float v_left_ret  = 0.0f;
float v_right_ret = 0.0f;
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  uint16_t rx_len;
	uart_frame_t frame;
	uint8_t tx_buf[32];
	uint16_t tx_len;
	//test git 
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
											xQueueSendToBack(motorCmdQueue, &cmd, 0);
										break;
								}
                
                case 0x02:
								{
									  ArmGive();
										uint8_t ack_data[] = {0x00};
										tx_len = serial_frame_build(0x82, ack_data, 1,tx_buf, sizeof(tx_buf));
										HAL_UART_Transmit(&huart6, tx_buf, tx_len, 100);
                    break;
								}   
								case 0x03:
								{
										ArmWave();
										uint8_t ack_data[] = {0x00};
										tx_len = serial_frame_build(0x83, ack_data, 1,tx_buf, sizeof(tx_buf));
										HAL_UART_Transmit(&huart6, tx_buf, tx_len, 100);
                    break;
								}
                case 0x04:
								{
										ArmRaise();
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
                case 0x06:
								{
										ArmRaise2();
										uint8_t ack_data[] = {0x00};
										tx_len = serial_frame_build(0x86, ack_data, 1,tx_buf, sizeof(tx_buf));
										HAL_UART_Transmit(&huart6, tx_buf, tx_len, 100);
                    break;
								}
								case 0x07:
								{
										ArmRaise3();
										uint8_t ack_data[] = {0x00};
										tx_len = serial_frame_build(0x87, ack_data, 1,tx_buf, sizeof(tx_buf));
										HAL_UART_Transmit(&huart6, tx_buf, tx_len, 100);
                    break;
								}
								case 0x08:
								{
										NeckUpDown();
										uint8_t ack_data[] = {0x00};
										tx_len = serial_frame_build(0x88, ack_data, 1,tx_buf, sizeof(tx_buf));
										HAL_UART_Transmit(&huart6, tx_buf, tx_len, 100);
                    break;
								}		
								case 0x09:
								{
										Armidle1();
										uint8_t ack_data[] = {0x00};
										tx_len = serial_frame_build(0x89, ack_data, 1,tx_buf, sizeof(tx_buf));
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
			osDelay(10);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_fun_ctrl_Task */
/**
* @brief Function implementing the fun_ctrl_task thread.
* @param argument: Not used
* @retval None
	
*/
int16_t ret_speed,ret_speed_r;
int32_t v_left_mm_s,v_right_mm_s;
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
  //neck_cmd_t cmd;
	float v_left, v_right;
	cmd_vel_t cmd;
	//GoAhead();
//	ArmGive();
//	osDelay(1000);
//	Armidle1();
//	osDelay(1000);
//	Armidle1();
//	osDelay(1000);
//	Armidle1();
//	osDelay(1000);
//	ArmRaise();
//	osDelay(1000);
//	ArmRaise1();
//	osDelay(1000);
//	ArmRaise2();
//	osDelay(1000);
//	ArmRaise3();
//	NeckUpDown();
//	osDelay(3000);
//	NeckUpDown();
		for(;;)
{			
    if (xQueueReceive(motorCmdQueue, &cmd, portMAX_DELAY) == pdPASS)
    {
        if (osMutexAcquire(uart_mutex, osWaitForever) == osOK)
        {
            float v_left_mps  = 0.0f;
            float v_right_mps = 0.0f;

            motor_set_get_speed_from_cmd(&cmd, 10, &v_left_mps, &v_right_mps);

            int32_t mileage_r = m0603a_lib_get_mileage_count(&motor_r_handle);
            int32_t cur_pos_r = m0603a_lib_get_cur_pos(&motor_r_handle);
            int32_t mileage_l = m0603a_lib_get_mileage_count(&motor_l_handle);
            int32_t cur_pos_l = m0603a_lib_get_cur_pos(&motor_l_handle);

            v_left_mm_s  = (int32_t)(v_left_mps  * 10000.0f);
            v_right_mm_s = (int32_t)(v_right_mps * 10000.0f);

            uint8_t send_data[24]; 
            ts = imu_time_base;

            send_data[0] = ts >> 24;
            send_data[1] = ts >> 16;
            send_data[2] = ts >> 8;
            send_data[3] = ts & 0xFF;

            send_data[4]  = mileage_l >> 24;
            send_data[5]  = mileage_l >> 16;
            send_data[6]  = mileage_l >> 8;
            send_data[7]  = mileage_l & 0xFF;
            send_data[8]  = cur_pos_l >> 8;
            send_data[9]  = cur_pos_l & 0xFF;

            send_data[10] = v_left_mm_s >> 24;
            send_data[11] = v_left_mm_s >> 16;
            send_data[12] = v_left_mm_s >> 8;
            send_data[13] = v_left_mm_s & 0xFF;

            send_data[14] = mileage_r >> 24;
            send_data[15] = mileage_r >> 16;
            send_data[16] = mileage_r >> 8;
            send_data[17] = mileage_r & 0xFF;
            send_data[18] = cur_pos_r >> 8;
            send_data[19] = cur_pos_r & 0xFF;

            send_data[20] = v_right_mm_s >> 24;
            send_data[21] = v_right_mm_s >> 16;
            send_data[22] = v_right_mm_s >> 8;
            send_data[23] = v_right_mm_s & 0xFF;

            uart_tx_frame_t tx;
            tx.len = serial_frame_build(0x81, send_data, sizeof(send_data), tx.buf, sizeof(tx.buf));
            xQueueSendToBack(uartTxQueue, &tx, 0);

            osMutexRelease(uart_mutex);
        }
    }
    osDelay(1);
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
	uint32_t last_led_tick = 0;
	uint32_t last_motor_tick = 0;
  ret = atk_ms6dsv_init();
  if (ret != 0)
  {
      printf("ATK-MS6DSV init failed!\r\n");
		  osDelay(200);
  }
	
	lsm6dsv16x_filt_settling_mask_t filt_settling_mask;
	lsm6dsv16x_data_ready_t drdy;
	uint8_t imu_data[16];
	int16_t data_raw_temperature;
  
		
	float acceleration_mg[3];
	float angular_rate_mdps[3];
  lsm6dsv16x_xl_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_240Hz);
  lsm6dsv16x_gy_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_240Hz);
    
  lsm6dsv16x_xl_full_scale_set(&atk_ms6dsv, LSM6DSV16X_2g);
  lsm6dsv16x_gy_full_scale_set(&atk_ms6dsv, LSM6DSV16X_2000dps);
   
	lsm6dsv16x_timestamp_set(&atk_ms6dsv, PROPERTY_ENABLE);

  filt_settling_mask.drdy = PROPERTY_ENABLE;
  filt_settling_mask.irq_xl = PROPERTY_ENABLE;
  filt_settling_mask.irq_g = PROPERTY_ENABLE;
  lsm6dsv16x_filt_settling_mask_set(&atk_ms6dsv, filt_settling_mask);
  lsm6dsv16x_filt_gy_lp1_set(&atk_ms6dsv, PROPERTY_ENABLE);
  lsm6dsv16x_filt_gy_lp1_bandwidth_set(&atk_ms6dsv, LSM6DSV16X_GY_ULTRA_LIGHT);
  lsm6dsv16x_filt_xl_lp2_set(&atk_ms6dsv, PROPERTY_ENABLE);
  lsm6dsv16x_filt_xl_lp2_bandwidth_set(&atk_ms6dsv, LSM6DSV16X_XL_STRONG);
	//uart_tx_frame_t tx;
	uint8_t tx_buf[64];
	uint16_t tx_len;
	static uint32_t last_wheel_tx_tick = 0;
	const uint32_t wheel_tx_interval = 50; // 50ms = 20Hz
  /* Infinite loop */
  for(;;)
  {
		//touch senior  todo
		  uint32_t now = osKernelGetTickCount();
		  if (now - last_led_tick >= 1000)
		  {
				  last_led_tick = now;
				  HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);
		  }
			
		  //imu
      lsm6dsv16x_flag_data_ready_get(&atk_ms6dsv, &drdy);
      if (drdy.drdy_xl)
      {
           lsm6dsv16x_acceleration_raw_get(&atk_ms6dsv, data_raw_acceleration);
           acceleration_mg[0] = lsm6dsv16x_from_fs2_to_mg(data_raw_acceleration[0]);
           acceleration_mg[1] = lsm6dsv16x_from_fs2_to_mg(data_raw_acceleration[1]);
           acceleration_mg[2] = lsm6dsv16x_from_fs2_to_mg(data_raw_acceleration[2]);
      }
     
      if (drdy.drdy_gy)
      {
           lsm6dsv16x_angular_rate_raw_get(&atk_ms6dsv, data_raw_angular_rate);
           angular_rate_mdps[0] = lsm6dsv16x_from_fs2000_to_mdps(data_raw_angular_rate[0]);
           angular_rate_mdps[1] = lsm6dsv16x_from_fs2000_to_mdps(data_raw_angular_rate[1]);
           angular_rate_mdps[2] = lsm6dsv16x_from_fs2000_to_mdps(data_raw_angular_rate[2]);
      }
			
			uint32_t imu_ts;
			lsm6dsv16x_timestamp_raw_get(&atk_ms6dsv, &imu_ts);
			imu_time_base = imu_ts;
			imu_data[0] = imu_ts >> 24;
			imu_data[1] = imu_ts >> 16;
			imu_data[2] = imu_ts >> 8;
			imu_data[3] = imu_ts & 0xFF;
		
			int16_t ax =(int16_t)acceleration_mg[0];
			int16_t ay =(int16_t)acceleration_mg[1];
			int16_t az =(int16_t)acceleration_mg[2];
			int16_t gx = (int16_t)angular_rate_mdps[0];
			int16_t gy = (int16_t)angular_rate_mdps[1];
			int16_t gz = (int16_t)angular_rate_mdps[2];
			imu_data[4]  = ax >> 8;
			imu_data[5]  = ax & 0xFF;
			imu_data[6]  = ay >> 8;
			imu_data[7]  = ay & 0xFF;
			imu_data[8]  = az >> 8;
			imu_data[9]  = az & 0xFF;
			imu_data[10] = gx >> 8;
			imu_data[11] = gx & 0xFF;
			imu_data[12] = gy >> 8;
			imu_data[13] = gy & 0xFF;
			imu_data[14] = gz >> 8;
			imu_data[15] = gz & 0xFF;
			tx.len = serial_frame_build(
						0x11,
					imu_data,
					sizeof(imu_data),
					tx.buf,
					sizeof(tx.buf)
			);
			
			xQueueSendToBack(uartTxQueue, &tx, 0);
			osDelay(5);
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
            while (neck_r_cur_angle != neck_r_target_angle ||
                   neck_pitch_cur  != neck_pitch_target)
            {
                Neck_Pitch_Update();
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
  }
  /* USER CODE END NeckTask */
}

/* USER CODE BEGIN Header_UartTxTask */
/**
* @brief Function implementing the uartTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UartTxTask */
void UartTxTask(void *argument)
{
  /* USER CODE BEGIN UartTxTask */
	uart_tx_frame_t frame;
  /* Infinite loop */
  for(;;)
  {
		if (xQueueReceive(uartTxQueue, &frame, portMAX_DELAY) == pdPASS)
    {
         HAL_UART_Transmit(&huart6, frame.buf, frame.len, 10);
    }
    osDelay(1);
  }
  /* USER CODE END UartTxTask */
}

/* USER CODE BEGIN Header_Wheel_Status_Task */
/**
* @brief Function implementing the wheel_status_ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Wheel_Status_Task */
void Wheel_Status_Task(void *argument)
{
  /* USER CODE BEGIN Wheel_Status_Task */
    /* Infinite loop */
//	int16_t start_angle = 70;
//  int16_t end_angle   = 70;
//	int16_t pitch_start = -30;
//  int16_t pitch_end   = 0;
//  neck_cmd_t cmd;
    for (;;)
    {
//	      cmd.angle       = end_angle;
//        cmd.pitch_angle = pitch_end;
//        xQueueSendToBack(neckQueue, &cmd, 0);
//        osDelay(1000); 

//        cmd.angle       = start_angle;
//        cmd.pitch_angle = pitch_start;
//        xQueueSendToBack(neckQueue, &cmd, 0);
        osDelay(1000);
    }
  /* USER CODE END Wheel_Status_Task */
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

