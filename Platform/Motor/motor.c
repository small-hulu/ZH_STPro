#include "motor.h"
#include <string.h>
#include "act.h"
#include <math.h>

static UART_HandleTypeDef *motor_uart;
static MotorFeedback_t motor_fb[2];
static MotorExtFeedback_t motor_ext_fb[2];
/* ============= CRC8 MAXIM ============= */
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

/* ============= 发送统一帧 ============= */
static void send_frame(uint8_t *f)
{
    f[9] = crc8_maxim(f, 9);
    HAL_UART_Transmit(motor_uart, f, 10, 50);
    HAL_Delay(2);
}

/* ============= 初始化 ============= */
void Motor_Init(UART_HandleTypeDef *huart)
{
    motor_uart = huart;
    memset(motor_fb, 0, sizeof(motor_fb));
}

/* ============= 基础控制 ============= */
void Motor_Enable(uint8_t id)
{
    uint8_t f[10] = {id, MOTOR_CMD_MODE, MOTOR_MODE_ENABLE};
    send_frame(f);
}

void Motor_Disable(uint8_t id)
{
    uint8_t f[10] = {id, MOTOR_CMD_MODE, MOTOR_MODE_DISABLE};
    send_frame(f);
}

void Motor_SetMode(uint8_t id, uint8_t mode)
{
    uint8_t f[10] = {id, MOTOR_CMD_MODE, mode};
    send_frame(f);
}

/* ============= 速度模式 ============= */
void Motor_SetSpeed(uint8_t id, int16_t rpm, uint8_t acc)
{
    uint8_t f[10] = {id, MOTOR_CMD_RUN};

    f[2] = rpm >> 8;
    f[3] = rpm & 0xFF;

    f[6] = acc;
    send_frame(f);
}

/* ============= 转矩模式 ============= */
void Motor_SetTorque(uint8_t id, int16_t ma, uint8_t acc)
{
    uint8_t f[10] = {id, MOTOR_CMD_RUN};

    f[2] = ma >> 8;
    f[3] = ma & 0xFF;

    f[6] = acc;
    send_frame(f);
}

/* ============= 位置模式 ============= */
void Motor_SetPosition(uint8_t id, int32_t pos, uint8_t acc)
{
    uint8_t f[10] = {id, MOTOR_CMD_RUN};

    f[2] = (pos >> 24) & 0xFF;
    f[3] = (pos >> 16) & 0xFF;
    f[4] = (pos >> 8) & 0xFF;
    f[5] = (pos) & 0xFF;

    f[6] = acc;
    send_frame(f);
}

/* ============= 读取反馈 ============= */
void Motor_RequestFeedback(uint8_t id)
{
    uint8_t f[10] = {id, MOTOR_CMD_READ};
    send_frame(f);
}
int16_t speed;
/* ============= 解析反馈帧（兼容 A1 / 65） ============= */
static void parse_frame(const uint8_t *f)
{
    if (crc8_maxim(f, 9) != f[9])
        return;

    uint8_t id = f[0];
    if (id < 1 || id > 2)
        return;

    uint8_t cmd = f[1];

    if (cmd == MOTOR_CMD_READ || cmd == 0x65)
    {
        motor_fb[id - 1].speed =
            (int16_t)((f[2] << 8) | f[3]);
        motor_fb[id - 1].current =
            (int16_t)((f[4] << 8) | f[5]);
        motor_fb[id - 1].temperature = f[7];
        motor_fb[id - 1].fault = f[8];
    }
    else if (cmd == 0x75)
    {
        motor_ext_fb[id - 1].lap_count =
            ((int32_t)f[2] << 24) |
            ((int32_t)f[3] << 16) |
            ((int32_t)f[4] << 8)  |
            ((int32_t)f[5]);

        motor_ext_fb[id - 1].position =
            ((uint16_t)f[6] << 8) | f[7];

        motor_ext_fb[id - 1].fault = f[8];
    }
}


/* ============= DMA-IDLE 进入此函数 ============= */
void Motor_ProcessRxData(uint8_t *buf, uint16_t len)
{
    if (len < 10) return;

    for (uint16_t i = 0; i + 10 <= len; i++)
    {
        parse_frame(&buf[i]);
    }

   
    xUART2.RxNum = 0;
}
void Motor_RequestExtFeedback(uint8_t id)
{
    if (id < 1 || id > 2) return;

    uint8_t f[10] = {0};

    f[0] = id;
    f[1] = MOTOR_CMD_MIL;

    // DATA[2~8] 默认 0
    f[9] = crc8_maxim(f, 9);

    HAL_UART_Transmit(motor_uart, f, 10, 50);
}

/* ============= 提供给用户接口 ============= */
uint8_t Motor_GetFeedback(MotorFeedback_t *fb, uint8_t id)
{
    if(id < 1 || id > 2) return 0;
    memcpy(fb, &motor_fb[id - 1], sizeof(MotorFeedback_t));
    return 1;
}

uint8_t Motor_GetExtFeedback(MotorExtFeedback_t *fb, uint8_t id)
{
    if (id < 1 || id > 2) return 0;

    memcpy(fb, &motor_ext_fb[id - 1], sizeof(MotorExtFeedback_t));
    return 1;
}
