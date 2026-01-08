
#ifndef _I2CC_H_
#define _I2CC_H_


#include "main.h"
//#include "data.h"
#include "stm32f4xx_hal_gpio.h"
/******************************************************************************************/
/* ?? ?? */
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_7
#define I2C_SDA_Pin GPIO_PIN_6

#define IIC_SCL_GPIO_PORT               I2C_SCL_GPIO_Port
#define IIC_SCL_GPIO_PIN                I2C_SCL_Pin
#define IIC_SDA_GPIO_PORT               I2C_SDA_GPIO_Port
#define IIC_SDA_GPIO_PIN                I2C_SDA_Pin
/******************************************************************************************/

/* IO?? */
#define IIC_SCL(x)        do{ x ? \
                              HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)       /* SCL */

#define IIC_SDA(x)        do{ x ? \
                              HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)       /* SDA */

#define IIC_READ_SDA     HAL_GPIO_ReadPin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN) /* ??SDA */


/* IIC?????? */
void iic_start(void);           /* ??IIC???? */
void iic_stop(void);            /* ??IIC???? */
void iic_ack(void);             /* IIC??ACK?? */
void iic_nack(void);            /* IIC???ACK?? */
uint8_t iic_wait_ack(void);     /* IIC??ACK?? */
void iic_send_byte(uint8_t txd);/* IIC?????? */
uint8_t iic_read_byte(unsigned char ack);/* IIC?????? */
void get_tempare(void);


#endif /* _I2CC_H_ */
