
#include "main.h"
#include "delay.h"

// achieve by DWT
void delay_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // ?? DWT ??
    DWT->CYCCNT = 0;                                 // ?????
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // ?? cycle counter
}

void delay_us(uint16_t us)
{
    uint32_t cycles = us * (SystemCoreClock / 1000000);  // 72MHz ?,cycles = us × 72
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}

void delay_ms(uint16_t ms)
{
    for(uint32_t i = 0;i < ms;i++)
        delay_us(1000);
}
