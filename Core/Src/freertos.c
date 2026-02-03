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
#include "../Platform/ADC/ads131m0x.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t ret;
int16_t data_raw_acceleration[3];
int16_t data_raw_angular_rate[3];

adc_channel_data myAdcRawData; // ADC采样原始值
adc_angle_data myAdcAngleData; // 翅膀角度值
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
	.priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for fun_ctrl_task */
osThreadId_t fun_ctrl_taskHandle;
const osThreadAttr_t fun_ctrl_task_attributes = {
	.name = "fun_ctrl_task",
	.stack_size = 512 * 4,
	.priority = (osPriority_t)osPriorityLow3,
};
/* Definitions for status_task */
osThreadId_t status_taskHandle;
const osThreadAttr_t status_task_attributes = {
	.name = "status_task",
	.stack_size = 512 * 4,
	.priority = (osPriority_t)osPriorityLow,
};
/* Definitions for neckTask */
osThreadId_t neckTaskHandle;
const osThreadAttr_t neckTask_attributes = {
	.name = "neckTask",
	.stack_size = 512 * 4,
	.priority = (osPriority_t)osPriorityLow,
};
/* Definitions for adcTask */
osThreadId_t adcTaskHandle;
const osThreadAttr_t adcTask_attributes = {
	.name = "adcTask",
	.stack_size = 512 * 4,
	.priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for adcDataQueue */
osMessageQueueId_t adcDataQueueHandle;
const osMessageQueueAttr_t adcDataQueue_attributes = {
	.name = "adcDataQueue"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
typedef struct
{
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
void AdcTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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

	/* Create the queue(s) */
	/* creation of adcDataQueue */
	adcDataQueueHandle = osMessageQueueNew(128, 24, &adcDataQueue_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */

	uartQueue = xQueueCreate(4, sizeof(uart_frame_t));
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
	// status_taskHandle = osThreadNew(Status_Task, NULL, &status_task_attributes);

	/* creation of neckTask */
	neckTaskHandle = osThreadNew(NeckTask, NULL, &neckTask_attributes);

	/* creation of adcTask */
	adcTaskHandle = osThreadNew(AdcTask, NULL, &adcTask_attributes);

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
uint8_t cur_cmd;
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
	/* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	uint16_t rx_len;
	uart_frame_t frame;
	uint8_t tx_buf[32];
	uint16_t tx_len;
	// test git
	for (;;)
	{
		if (xQueueReceive(uartQueue, &frame, portMAX_DELAY) == pdPASS)
		{
			// Unpacking
			// printf("%s", frame.data);
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

					Motor_SetSpeed_FromCmd(&cmd, 18);
					// send cmd and v
					uint8_t ack_data[] = {0x00};
					tx_len = serial_frame_build(0x81, ack_data, 1, tx_buf, sizeof(tx_buf));
					HAL_UART_Transmit(&huart6, tx_buf, tx_len, 100);
					break;
				}

				case 0x02:
				{
					ArmGive();
					uint8_t ack_data[] = {0x00};
					tx_len = serial_frame_build(0x82, ack_data, 1, tx_buf, sizeof(tx_buf));
					HAL_UART_Transmit(&huart6, tx_buf, tx_len, 100);
					break;
				}
				case 0x03:
				{
					ArmWave();
					uint8_t ack_data[] = {0x00};
					tx_len = serial_frame_build(0x83, ack_data, 1, tx_buf, sizeof(tx_buf));
					HAL_UART_Transmit(&huart6, tx_buf, tx_len, 100);
					break;
				}
				case 0x04:
				{
					ArmRaise();
					uint8_t ack_data[] = {0x00};
					tx_len = serial_frame_build(0x84, ack_data, 1, tx_buf, sizeof(tx_buf));
					HAL_UART_Transmit(&huart6, tx_buf, tx_len, 100);
					break;
				}
				case 0x05:
				{
					// ack
					neck_cmd.angle = (int16_t)((pkt.data[0] << 8) | pkt.data[1]);
					neck_cmd.pitch_angle = (int16_t)((pkt.data[2] << 8) | pkt.data[3]);

					xQueueSendToBack(neckQueue, &neck_cmd, 0);

					uint8_t ack_data[] = {0x00};
					tx_len = serial_frame_build(0x85, ack_data, 1, tx_buf, sizeof(tx_buf));
					HAL_UART_Transmit(&huart6, tx_buf, tx_len, 100);
					break;
				}
				case 0x06:
				{
					ArmRaise2();
					uint8_t ack_data[] = {0x00};
					tx_len = serial_frame_build(0x86, ack_data, 1, tx_buf, sizeof(tx_buf));
					HAL_UART_Transmit(&huart6, tx_buf, tx_len, 100);
					break;
				}
				case 0x07:
				{
					ArmRaise3();
					uint8_t ack_data[] = {0x00};
					tx_len = serial_frame_build(0x87, ack_data, 1, tx_buf, sizeof(tx_buf));
					HAL_UART_Transmit(&huart6, tx_buf, tx_len, 100);
					break;
				}
				case 0x08:
				{
					NeckUpDown();
					uint8_t ack_data[] = {0x00};
					tx_len = serial_frame_build(0x88, ack_data, 1, tx_buf, sizeof(tx_buf));
					HAL_UART_Transmit(&huart6, tx_buf, tx_len, 100);
					break;
				}
				case 0x09:
				{
					Armidle1();
					uint8_t ack_data[] = {0x00};
					tx_len = serial_frame_build(0x89, ack_data, 1, tx_buf, sizeof(tx_buf));
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

	// neck_init
	Neckinit();

	fsm_lib_start(&main_ctrl_fsm_handle);
	/* Infinite loop */
	int16_t start_angle = 70;
	int16_t end_angle = 140;
	int16_t pitch_start = 0;
	int16_t pitch_end = 60;
	neck_cmd_t cmd;
	Stop();
	// GoAhead();
	ArmWave();
	Armidle1();
	osDelay(1000);
	Armidle1();
	osDelay(1000);
	Armidle1();
	ArmRaise1();
	osDelay(1000);
	ArmRaise2();
	osDelay(1000);
	ArmRaise3();
	NeckUpDown();
	osDelay(3000);
	NeckUpDown();
	for (;;)
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
	uint32_t last_led_tick = 0;

	ret = atk_ms6dsv_init();

	//    ret = atk_ms6dsv_init();

	if (ret != 0)
	{

		printf("ATK-MS6DSV init failed!\r\n");
		osDelay(200);
	}

	lsm6dsv16x_filt_settling_mask_t filt_settling_mask;
	lsm6dsv16x_data_ready_t drdy;
	uint8_t imu_data[12];
	int16_t data_raw_temperature;

	uint8_t i2ctimes;

	float acceleration_mg[3];
	float angular_rate_mdps[3];
	lsm6dsv16x_xl_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_60Hz);
	lsm6dsv16x_gy_data_rate_set(&atk_ms6dsv, LSM6DSV16X_ODR_AT_60Hz);

	lsm6dsv16x_xl_full_scale_set(&atk_ms6dsv, LSM6DSV16X_2g);
	lsm6dsv16x_gy_full_scale_set(&atk_ms6dsv, LSM6DSV16X_2000dps);

	filt_settling_mask.drdy = PROPERTY_ENABLE;
	filt_settling_mask.irq_xl = PROPERTY_ENABLE;
	filt_settling_mask.irq_g = PROPERTY_ENABLE;
	lsm6dsv16x_filt_settling_mask_set(&atk_ms6dsv, filt_settling_mask);
	lsm6dsv16x_filt_gy_lp1_set(&atk_ms6dsv, PROPERTY_ENABLE);
	lsm6dsv16x_filt_gy_lp1_bandwidth_set(&atk_ms6dsv, LSM6DSV16X_GY_ULTRA_LIGHT);
	lsm6dsv16x_filt_xl_lp2_set(&atk_ms6dsv, PROPERTY_ENABLE);
	lsm6dsv16x_filt_xl_lp2_bandwidth_set(&atk_ms6dsv, LSM6DSV16X_XL_STRONG);
	uint8_t tx_buf[64];
	uint16_t tx_len;
	/* Infinite loop */
	for (;;)
	{
		// touch senior  todo
		uint32_t now = osKernelGetTickCount();
		if (now - last_led_tick >= 1000)
		{
			last_led_tick = now;
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);
		}
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
		if (++i2ctimes == 20)
		{
			i2ctimes = 0;
			int16_t ax = (int16_t)acceleration_mg[0];
			int16_t ay = (int16_t)acceleration_mg[1];
			int16_t az = (int16_t)acceleration_mg[2];
			int16_t gx = (int16_t)angular_rate_mdps[0];
			int16_t gy = (int16_t)angular_rate_mdps[1];
			int16_t gz = (int16_t)angular_rate_mdps[2];
			imu_data[0] = ax >> 8;
			imu_data[1] = ax & 0xFF;
			imu_data[2] = ay >> 8;
			imu_data[3] = ay & 0xFF;
			imu_data[4] = az >> 8;
			imu_data[5] = az & 0xFF;
			imu_data[6] = gx >> 8;

			imu_data[7] = gx & 0xFF;
			imu_data[8] = gy >> 8;
			imu_data[9] = gy & 0xFF;
			imu_data[10] = gz >> 8;
			imu_data[11] = gz & 0xFF;

			tx_len = serial_frame_build(
				0x11,
				imu_data,
				sizeof(imu_data),
				tx_buf,
				sizeof(tx_buf));
			HAL_UART_Transmit(&huart6, tx_buf, tx_len, 50);
		}

		osDelay(10);
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
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 150);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 150);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 150);
	for (;;)
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
				vTaskDelay(pdMS_TO_TICKS(20));
			}
		}
	}
	/* USER CODE END NeckTask */
}

/* USER CODE BEGIN Header_AdcTask */
/**
 * @brief Function implementing the adcTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_AdcTask */
void AdcTask(void *argument)
{
	/* USER CODE BEGIN AdcTask */

	uint16_t regValue = 0;
	adc_angle_data angleData;
	adc_channel_data recvData;

	int64_t sumCh[6] = {0};
	uint16_t sampleCount = 0;
	const uint16_t AVG_COUNT = 50;

	/* 等待系统稳定 */
	vTaskDelay(pdMS_TO_TICKS(100));

	/* ADC 初始化 */
	printf("Starting ADC Startup...\r\n");
	if (!adcStartup())
	{
		printf("Error: ADC Startup Failed!\r\n");
		vTaskSuspend(NULL);
	}
	printf("ADC Initialized.\r\n");

	/* 等待 ADC 稳定 */
	vTaskDelay(pdMS_TO_TICKS(500));

	/* 检查通信 */
	regValue = readSingleRegister(ID_ADDRESS);
	printf("Read ID Register: 0x%04X\r\n", regValue);
	if (regValue == 0x0000 || regValue == 0xFFFF)
	{
		printf("Error: SPI Communication Failed!\r\n");
		vTaskSuspend(NULL);
	}

	regValue = readSingleRegister(STATUS_ADDRESS);
	printf("Read STATUS Register: 0x%04X\r\n", regValue);

	/* 清空队列，丢弃初始化期间可能产生的数据 */
	while (osMessageQueueGet(adcDataQueueHandle, &recvData, NULL, 0) == osOK)
	{
		// 清空
	}
	printf("ADC Task Running, Sampling...\r\n");
	/* Infinite loop */
	for (;;)
	{
		osStatus_t status = osMessageQueueGet(adcDataQueueHandle, &recvData, NULL, pdMS_TO_TICKS(100));

		if (status == osOK)
		{
			/* 检查 response 状态位（可选） */
			if (recvData.response & 0x0001) // 根据数据手册检查错误位
			{
				// 可以记录错误计数
			}

			/* 累加数据 */
			sumCh[0] += recvData.channel0;
			sumCh[1] += recvData.channel1;
			sumCh[2] += recvData.channel2;
			sumCh[3] += recvData.channel3;
			sumCh[4] += recvData.channel4;
			sumCh[5] += recvData.channel5;
			sampleCount++;

			/* 达到平均次数，输出结果 */
			if (sampleCount >= AVG_COUNT)
			{
				adc_channel_data avgData;
				avgData.channel0 = (int32_t)(sumCh[0] / AVG_COUNT);
				avgData.channel1 = (int32_t)(sumCh[1] / AVG_COUNT);
				avgData.channel2 = (int32_t)(sumCh[2] / AVG_COUNT);
				avgData.channel3 = (int32_t)(sumCh[3] / AVG_COUNT);
				avgData.channel4 = (int32_t)(sumCh[4] / AVG_COUNT);
				avgData.channel5 = (int32_t)(sumCh[5] / AVG_COUNT);
				convertToAngle(&avgData, &angleData);
				printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
					   angleData.angle[0], angleData.angle[1],
					   angleData.angle[2], angleData.angle[3],
					   angleData.angle[4], angleData.angle[5]);
				for (int i = 0; i < 6; i++)
				{
					sumCh[i] = 0;
				}
				sampleCount = 0;
			}
		}
		else if (status == osErrorTimeout)
		{
			/* 超时，可能 ADC 停止工作 */
			printf("Warning: ADC Data Timeout!\r\n");

			/* 检查 ADC 状态 */
			regValue = readSingleRegister(STATUS_ADDRESS);
			printf("STATUS: 0x%04X\r\n", regValue);
		}
		else
		{
			/* 其他错误 */
			printf("Error: Queue Get Failed! Status: %d\r\n", status);
		}

		/* 检查队列使用情况 */
		uint32_t queueCount = osMessageQueueGetCount(adcDataQueueHandle);
		if (queueCount > 100) // 队列快满了
		{
			printf("Warning: Queue almost full! Count: %lu\r\n", queueCount);
		}
	}

	/* USER CODE END AdcTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
 * @brief   fun_ctrl fsm
 */
fsm_lib_return main_ctrl_fsm(fsm_lib_ctrl_handle *handle)
{
	enum
	{
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

	switch (handle->state)
	{
		fsm_lib_state(idle)
		{
			printf("main_ctrl_fsm idle\r\n");
			fsm_lib_release(handle, fsm_rt_idle);
		}
		break;

		fsm_lib_state(start)
		{
			printf("main_ctrl_fsm start\r\n");
			fsm_lib_update_to(handle, seft_tset);
		}
		break;

		fsm_lib_state(end)
		{
			fsm_lib_transfer_to(handle, idle, fsm_rt_cpl);
		}
		break;

		fsm_lib_state(error) {}
		break;
		fsm_lib_state(seft_tset)
		{
			printf("main_ctrl_fsm seft_tset\r\n");
			fsm_lib_update_to(handle, running);
		}
		break;
		fsm_lib_state(running)
		{
			printf("main_ctrl_fsm running\r\n");
			fsm_lib_release(handle, fsm_rt_running);
		}
		break;
		fsm_lib_state(exit)
		{
		}
		break;
		fsm_lib_state(func_error)
		{
		}
		break;
	}

	return fsm_rt_running;
}
/* USER CODE END Application */
