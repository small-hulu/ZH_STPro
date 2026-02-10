/***************************************************************************
 * Copyright  2026 xushaowei
 * All right reserved. See COPYRIGHT for detailed Information.
 *
 * @file        motor_ctrl.c
 *
 * @author     xushaowei
 * @brief      
 *
 * @Email      
 *
 * @date       2025/12/22
 * @version    0.0.1
 ***************************************************************************/
 
#include "motor_ctrl.h"
#include "cmsis_os.h"
/**
 * @brief  moto l info
 */
m0603a_info motor_l_info = {.uart_handle = &huart2};
 
 
 /**
 * @brief  moto l handle
 */
m0603a_lib_handle motor_l_handle;
 
 
 
 /**
 * @brief  moto l mutex attr
 */
 const osMutexAttr_t moto_l_mutex_attr = {
    .name = "moto_l_mutex",
    .attr_bits = osMutexPrioInherit | osMutexRecursive | osMutexRobust
};

/**
 * @brief  moto l mutex
 */
osMutexId_t moto_l_mutex = NULL;
 
 /**
 * @brief  moto l lock method
 */
static inline int moto_l_lock(uint8_t lock) {
    if (lock) {
        return osMutexAcquire(moto_l_mutex, osWaitForever);
    } else {
        return osMutexRelease(moto_l_mutex);
    }
}

/**
 * @brief  motor l info
 */
 m0603a_info motor_r_info = {.uart_handle = &huart2};
 
 
 /**
 * @brief  motor l handle
 */
 m0603a_lib_handle motor_r_handle;
 
 
/**
 * @brief  motor l init
 */
void moto_l_init()
{
		moto_l_mutex = osMutexNew(&moto_l_mutex_attr);
    m0603a_lib_handle_init(&motor_l_handle, &motor_l_info, 0x01, moto_l_lock);
		m0603a_lib_set_motor_en(&motor_l_handle);
		m0603a_lib_set_mode(&motor_l_handle,MOTOR_MODE_SPEED);
		m0603a_lib_set_get_speed(&motor_l_handle,0, 5);
}
/**
 * @brief  motor r init
 */
void moto_r_init()
{
		moto_l_mutex = osMutexNew(&moto_l_mutex_attr);
    m0603a_lib_handle_init(&motor_r_handle, &motor_r_info, 0x02, moto_l_lock);
		m0603a_lib_set_motor_en(&motor_r_handle);
		m0603a_lib_set_mode(&motor_r_handle,MOTOR_MODE_SPEED);
		m0603a_lib_set_get_speed(&motor_r_handle,0, 5);
}

static inline float lowpass(float last, float now, float alpha)
{
    return alpha * last + (1.0f - alpha) * now;
}

/**
 * @brief  motor_set_get_speed_from_cmd
 */
float v_left, v_right;
int16_t speed_left,speed_right;
float v_left_filt  = 0.0f;
float v_right_filt = 0.0f;
int16_t rpm_l_cmd,rpm_r_cmd;
#define SPEED_FEEDBACK_AVG_N 5

void motor_set_get_speed_from_cmd(const cmd_vel_t *cmd,
                                  uint8_t acc,
                                  float *v_left_out,
                                  float *v_right_out)
{
    int32_t v_l_sum = 0;
    int32_t v_r_sum = 0;

    float linear_x_m_s    = cmd->linear_x_mm_s   / 1000.0f;
    float angular_z_rad_s = cmd->angular_z_mrad  / 1000.0f;

    v_left  = linear_x_m_s - angular_z_rad_s * (WHEEL_BASE_M / 2.0f);
    v_right = linear_x_m_s + angular_z_rad_s * (WHEEL_BASE_M / 2.0f);

    rpm_l_cmd = (int16_t)(v_left  * MS_TO_RPM * 10.0f);
    rpm_r_cmd = (int16_t)(v_right * MS_TO_RPM * 10.0f);
    rpm_r_cmd = -rpm_r_cmd;

    for (int i = 0; i < SPEED_FEEDBACK_AVG_N; i++) {
        int16_t speed_l_raw = m0603a_lib_set_get_speed(&motor_l_handle, rpm_l_cmd, acc);
        int16_t speed_r_raw = m0603a_lib_set_get_speed(&motor_r_handle, rpm_r_cmd, acc);
				motor_l_handle.status.rpm = speed_l_raw / 10.0f;
				motor_r_handle.status.rpm = speed_r_raw / 10.0f;
        v_l_sum += speed_l_raw;
        v_r_sum += speed_r_raw;
        osDelay(1);  
    }

    float v_l_avg_raw  = (float)v_l_sum / SPEED_FEEDBACK_AVG_N; 
    float v_r_avg_raw  = (float)v_r_sum / SPEED_FEEDBACK_AVG_N; 

		float v_left_avg_mps  = (v_l_avg_raw * RPM_TO_MPS) / 10.0f;
    float v_right_avg_mps = (v_r_avg_raw * RPM_TO_MPS) / 10.0f;

    if (v_left_out) {
        *v_left_out = v_left_avg_mps;
    }
    if (v_right_out) {
        *v_right_out = v_right_avg_mps;
    }
}
