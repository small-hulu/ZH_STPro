

#include "atk_ms6dsv_iic.h"
#include "delay.h"

/**
 * @brief       IIC????,????IIC????
 * @param       ?
 * @retval      ?
 */
static void iic_delay(void)
{
    delay_us(2);    /* 2us???, ?????250Khz?? */
}

/**
 * @brief       ??IIC????
 * @param       ?
 * @retval      ?
 */
void iic_start(void)
{
    IIC_SDA(1);
    IIC_SCL(1);
    iic_delay();
    IIC_SDA(0);     /* START??: ?SCL???, SDA?????, ?????? */
    iic_delay();
    IIC_SCL(0);     /* ??I2C??,????????? */
    iic_delay();
}

/**
 * @brief       ??IIC????
 * @param       ?
 * @retval      ?
 */
void iic_stop(void)
{
    IIC_SDA(0);     /* STOP??: ?SCL???, SDA?????, ?????? */
    iic_delay();
    IIC_SCL(1);
    iic_delay();
    IIC_SDA(1);     /* ??I2C?????? */
    iic_delay();
}

/**
 * @brief       ????????
 * @param       ?
 * @retval      1,??????
 *              0,??????
 */
uint8_t iic_wait_ack(void)
{
    uint8_t waittime = 0;
    uint8_t rack = 0;

    IIC_SDA(1);     /* ????SDA?(??????????SDA?) */
    iic_delay();
    IIC_SCL(1);     /* SCL=1, ????????ACK */
    iic_delay();

    while (IIC_READ_SDA)    /* ???? */
    {
        waittime++;

        if (waittime > 250)
        {
            iic_stop();
            rack = 1;
            break;
        }
    }

    IIC_SCL(0);     /* SCL=0, ??ACK?? */
    iic_delay();
    return rack;
}

/**
 * @brief       ??ACK??
 * @param       ?
 * @retval      ?
 */
void iic_ack(void)
{
    IIC_SDA(0);     /* SCL 0 -> 1  ? SDA = 0,???? */
    iic_delay();
    IIC_SCL(1);     /* ?????? */
    iic_delay();
    IIC_SCL(0);
    iic_delay();
    IIC_SDA(1);     /* ????SDA? */
    iic_delay();
}

/**
 * @brief       ???ACK??
 * @param       ?
 * @retval      ?
 */
void iic_nack(void)
{
    IIC_SDA(1);     /* SCL 0 -> 1  ? SDA = 1,????? */
    iic_delay();
    IIC_SCL(1);     /* ?????? */
    iic_delay();
    IIC_SCL(0);
    iic_delay();
}

/**
 * @brief       IIC??????
 * @param       data: ??????
 * @retval      ?
 */
void iic_send_byte(uint8_t data)
{
    uint8_t t;

    for (t = 0; t < 8; t++)
    {
        IIC_SDA((data & 0x80) >> 7);    /* ????? */
        iic_delay();
        IIC_SCL(1);
        iic_delay();
        IIC_SCL(0);
        data <<= 1;     /* ??1?,??????? */
    }
    IIC_SDA(1);         /* ????, ????SDA? */
}

/**
 * @brief       IIC??????
 * @param       ack:  ack=1?,??ack; ack=0?,??nack
 * @retval      ??????
 */
uint8_t iic_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0;

    for (i = 0; i < 8; i++ )    /* ??1????? */
    {
        receive <<= 1;  /* ?????,???????????? */
        IIC_SCL(1);
        iic_delay();

        if (IIC_READ_SDA)
        {
            receive++;
        }

        IIC_SCL(0);
        iic_delay();
    }

    if (!ack)
    {
        iic_nack();     /* ??nACK */
    }
    else
    {
        iic_ack();      /* ??ACK */
    }

    return receive;
}
