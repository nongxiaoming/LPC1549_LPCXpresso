#include <rtthread.h>
#include "board.h"

/**
* BLUE_LED  <---> PIO1_1
* GREEN_LED <---> PIO0_3
* RED_LED   <---> PIO0_25
**/
int led_hw_init(void)
{
	/* enable clock for GPIO0 and GPIO1 port */
  LPC_SYSCON->SYSAHBCLKCTRL0 |= (1UL << 14) | (1UL << 15);
  /* configure GPIO as output */
  LPC_GPIO_PORT->DIR[0]  |= (0x01<<3) | (0x01<<25);
  LPC_GPIO_PORT->SET[0]  |= (0x01<<3) | (0x01<<25);
  LPC_GPIO_PORT->DIR[1]  |= (0x01<<1);
  LPC_GPIO_PORT->SET[1]  |= (0x01<<1);
	
    return 0;
}
INIT_DEVICE_EXPORT(rt_led_hw_init);

