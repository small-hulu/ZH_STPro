#ifndef BSP_DELAY_H_
#define BSP_DELAY_H_

#include "./main.h"
void delay_init(void);       //init
void delay_ms(uint16_t nms);            /* delay n ms */
void delay_us(uint16_t nus);            /* delay n us */

#endif /* BSP_DELAY_H_ */
