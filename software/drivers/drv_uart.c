/*
 * File      : drv_uart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009-2013 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-01-18     xiaonong      The first version for LPC15xx
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "board.h"

struct lpc_uart
{
    LPC_USART0_Type *UART;
    IRQn_Type UART_IRQn;
};

static rt_err_t lpc_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct lpc_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct lpc_uart *)serial->parent.user_data;


  /* 115200 baud @ 12MHz */
  LPC_SYSCON->UARTCLKDIV = 6;                   /* UART clock =  PCLK / 6     */
  LPC_SYSCON->FRGCTRL    = 0x15FF;
  uart->UART->BRG  = ((12000000UL / 16 / cfg->baud_rate) -1);

  uart->UART->CFG  = ((1UL << 0) |                    /* Enable USART               */
                (1UL << 2) |                    /* 8 data bits                */
                (0UL << 4) |                    /* no parity                  */
                (0UL << 6)  );                  /* 1 stop bit                 */


    return RT_EOK;
}

static rt_err_t lpc_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct lpc_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct lpc_uart *)serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        uart->UART->INTENCLR &= ~0x01;
        break;
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        uart->UART->INTENSET |= 0x01;
        break;
    }

    return RT_EOK;
}

static int lpc_putc(struct rt_serial_device *serial, char c)
{
    struct lpc_uart *uart;

    uart = (struct lpc_uart *)serial->parent.user_data;
    while (!(uart->UART->STAT & (1UL << 2)));
    uart->UART->TXDATA = c;

    return 1;
}

static int lpc_getc(struct rt_serial_device *serial)
{
    struct lpc_uart *uart;

    uart = (struct lpc_uart *)serial->parent.user_data;
    if (uart->UART->STAT & 0x01)
        return (uart->UART->RXDATA);
    else
        return -1;
}

static const struct rt_uart_ops lpc_uart_ops =
{
    lpc_configure,
    lpc_control,
    lpc_putc,
    lpc_getc,
};

#ifdef RT_USING_UART0
/* UART0 device driver structure */
struct lpc_uart uart0 =
{
    LPC_USART0,
    UART0_IRQn,
};
struct rt_serial_device serial0;

void UART0_IRQHandler(void)
{
    volatile  uint32_t IIR, tmp;

    /* enter interrupt */
    rt_interrupt_enter();

    IIR = LPC_USART0->INTSTAT;
    IIR &= 0x0e;
    switch (IIR)
    {

    case 0x04:
        rt_hw_serial_isr(&serial0, RT_SERIAL_EVENT_RX_IND);
        break;
    case 0x06:
       // tmp = LPC_UART0->LSR;
        break;
    case 0x0c:
        rt_hw_serial_isr(&serial0, RT_SERIAL_EVENT_RX_IND);
        break;
    default :
       // tmp = LPC_UART0->LSR;
        break;
    }
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif
#ifdef RT_USING_UART2
/* UART2 device driver structure */
struct lpc_uart uart2 =
{
    LPC_USART2,
    UART2_IRQn,
};
struct rt_serial_device serial2;

void UART2_IRQHandler(void)
{
    volatile  uint32_t IIR, tmp;


    /* enter interrupt */
    rt_interrupt_enter();

    IIR = LPC_UART2->IIR;
    IIR &= 0x0e;
    switch (IIR)
    {

    case 0x04:
        rt_hw_serial_isr(&serial2, RT_SERIAL_EVENT_RX_IND);
        break;
    case 0x06:
        tmp = LPC_UART2->LSR;
        break;
    case 0x0c:
        rt_hw_serial_isr(&serial2, RT_SERIAL_EVENT_RX_IND);
        break;
    default :
        tmp = LPC_UART2->LSR;
        break;
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif
#ifdef RT_USING_UART4
/* UART4 device driver structure */
struct lpc_uart uart4 =
{
    LPC_UART4,
    UART4_IRQn,
};
struct rt_serial_device serial4;

void UART4_IRQHandler(void)
{
    volatile  uint32_t IIR, tmp;


    /* enter interrupt */
    rt_interrupt_enter();

    IIR = LPC_UART4->IIR;
    IIR &= 0x0e;
    switch (IIR)
    {

    case 0x04:
        rt_hw_serial_isr(&serial4, RT_SERIAL_EVENT_RX_IND);
        break;
    case 0x06:
        tmp = LPC_UART4->LSR;
        break;
    case 0x0c:
        rt_hw_serial_isr(&serial4, RT_SERIAL_EVENT_RX_IND);
        break;
    default :
        tmp = LPC_UART4->LSR;
        break;
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

void rt_hw_uart_init(void)
{
    struct lpc_uart *uart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

#ifdef RT_USING_UART0
    uart = &uart0;

    serial0.ops    = &lpc_uart_ops;
    serial0.config = config;
    serial0.parent.user_data = uart;
    /*
     * Initialize USART0 pin connect
     * P0.18: U0_TXD
     * P0.13: U0_RXD
     */
     /* configure PINs GPIO0.18, GPIO0.13 for UART */
    LPC_SYSCON->SYSAHBCLKCTRL0 |= ((1UL << 14) |  /* enable clock for GPIO0     */
                                 (1UL << 12)  );/* enable clock for SWM       */

    LPC_SWM->PINASSIGN0 &= ~((0xFF <<  0) |       /* clear PIN assign UART0_TXD */
                           (0xFF <<  8)  );     /* clear PIN assign UART0_RXD */
    LPC_SWM->PINASSIGN0 |=  ((  18 <<  0) |       /* PIN assign UART0_TXD  P0.18 */
                           (  13 <<  8)  );     /* PIN assign UART0_RXD  P0.13 */

     /* configure UART0 */
     LPC_SYSCON->SYSAHBCLKCTRL1 |=  (1UL << 17);   /* Enable clock to UART0      */

    /* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(uart->UART_IRQn, ((0x01 << 3) | 0x01));

    /* Enable Interrupt for UART channel */
    NVIC_EnableIRQ(uart->UART_IRQn);

    /* register UART0 device */
    rt_hw_serial_register(&serial0, "uart0",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM,
                          uart);
#endif

#ifdef RT_USING_UART2
    uart = &uart2;

    serial2.ops    = &lpc_uart_ops;
    serial2.config = config;
    serial2.parent.user_data = uart;
    /*
     * Initialize UART2 pin connect
     * P2.8: U2_TXD
     * P0.11: U2_RXD
     */
    LPC_IOCON->P2_8 &= ~0x07;
    LPC_IOCON->P0_11 &= ~0x07;
    LPC_IOCON->P2_8 |= 0x02;
    LPC_IOCON->P0_11 |= 0x01;

    /* enable the uart2 power and clock */
    LPC_SC->PCONP |= 0x01 << 24;
    /* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(uart->UART_IRQn, ((0x01 << 3) | 0x01));

    /* Enable Interrupt for UART channel */
    NVIC_EnableIRQ(uart->UART_IRQn);

    /* register UART2 device */
    rt_hw_serial_register(&serial2, "uart2",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM,
                          uart);
#endif

#ifdef RT_USING_UART4
    uart = &uart4;

    serial4.ops    = &lpc_uart_ops;
    serial4.config = config;

    /*
     * Initialize UART2 pin connect
     * P5.4: U2_TXD
     * P5.3: U2_RXD
     */
    LPC_IOCON->P5_4 &= ~0x07;
    LPC_IOCON->P5_3 &= ~0x07;
    LPC_IOCON->P5_4 |= 0x04;
    LPC_IOCON->P5_3 |= 0x04;

    /* enable the uart4 power and clock */
    LPC_SC->PCONP |= 0x01 << 8;
    /* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(uart->UART_IRQn, ((0x01 << 3) | 0x01));

    /* Enable Interrupt for UART channel */
    NVIC_EnableIRQ(uart->UART_IRQn);

    /* register UART2 device */
    rt_hw_serial_register(&serial4, "uart4",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM,
                          uart);
#endif
}
