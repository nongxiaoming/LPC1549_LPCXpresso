/*
 * File      : drv_spi.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-01-21     xiaonong      The first version for LPC15xx
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"

#ifdef RT_USING_SPI
#include "drv_spi.h"

/* private rt-thread spi ops function */
static rt_err_t configure(struct rt_spi_device *device, struct rt_spi_configuration *configuration);
static rt_uint32_t xfer(struct rt_spi_device *device, struct rt_spi_message *message);

static struct rt_spi_ops lpc_spi_ops =
{
    configure,
    xfer
};

#ifdef SPI_USE_DMA
static uint8_t dummy = 0xFF;
static void DMA_RxConfiguration(struct lpc_spi_bus *lpc_spi_bus,
                                const void *send_addr,
                                void *recv_addr,
                                rt_size_t size)
{

}
#endif

static rt_err_t configure(struct rt_spi_device *device,
                          struct rt_spi_configuration *configuration)
{
    struct lpc_spi_bus *spi_bus = (struct lpc_spi_bus *)device->bus;
	
	  /* Disable spi device */
	  spi_bus->SPI->CFG &= ~(0x01<<0);
    /* data_width */
    if (configuration->data_width > 3 && configuration->data_width <= 16)
    {
        //spi_bus->SPI->CR0 = (configuration->data_width - 1);

    }
    else
    {
        return RT_EIO;
    }
    /* baudrate */
    {
        uint16_t div_val=0;
        div_val = SystemCoreClock/configuration->max_hz;

        spi_bus->SPI->DIV = div_val;
    }

    /* CPOL */
    if (configuration->mode & RT_SPI_CPOL)
    {
        spi_bus->SPI->CFG |= (0x01 << 5);
    }
    else
    {
        spi_bus->SPI->CFG &= ~(0x01 << 5);
    }
    /* CPHA */
    if (configuration->mode & RT_SPI_CPHA)
    {
        spi_bus->SPI->CFG |= (0x01 << 4);
    }
    else
    {
        spi_bus->SPI->CFG &= ~(0x01 << 4);
    }
//    /*Clear the RxFIFO*/
//    {
//        uint8_t i;
//        uint16_t temp = temp;
//        for (i = 0; i < 8; i++)
//        {
//            temp = spi_bus->SPI->DR;
//        }
//    }
    /* Enable SPI_MASTER */
     spi_bus->SPI->CFG |= (0x05);

    return RT_EOK;
}

static rt_uint32_t xfer(struct rt_spi_device *device, struct rt_spi_message *message)
{
    struct lpc_spi_bus *spi_bus = (struct lpc_spi_bus *)device->bus;
    struct rt_spi_configuration *config = &device->config;
    struct lpc_spi_cs *spi_cs = device->parent.user_data;
    rt_uint32_t size = message->length;

    /* take CS */
    if (message->cs_take)
    {
      LPC_GPIO_PORT->CLR[spi_cs->port] |= (0x01 << spi_cs->pin);
    }

    {
        if (config->data_width <= 8)
        {
            const rt_uint8_t *send_ptr = message->send_buf;
            rt_uint8_t *recv_ptr = message->recv_buf;
            //  rt_kprintf("size =%d",size);
            while (size--)
            {
                rt_uint8_t data = 0x00;

                if (send_ptr != RT_NULL)
                {
                    data = *send_ptr++;
                }

                //Wait until the transmit buffer is empty
                while ((spi_bus->SPI->SR & ((0x01 << 1) | (0x01 << 4))) != 0x02);
                // Send the byte
                spi_bus->SPI->TXDAT = data;
                //Wait until a data is received
                while ((spi_bus->SPI->SR & ((0x01 << 2) | (0x01 << 4))) != 0x04);
                // Get the received data
                data = spi_bus->SPI->RXDAT;
                if (recv_ptr != RT_NULL)
                {
                    *recv_ptr++ = data;
                }
            }
        }
        else if (config->data_width <= 16)
        {
            const rt_uint16_t *send_ptr = message->send_buf;
            rt_uint16_t *recv_ptr = message->recv_buf;

            while (size--)
            {
                rt_uint16_t data = 0xFF;

                if (send_ptr != RT_NULL)
                {
                    data = *send_ptr++;
                }

                //Wait until the transmit buffer is empty
                while ((spi_bus->SPI->SR & ((0x01 << 1) | (0x01 << 4))) != 0x02);
                // Send the byte
                spi_bus->SPI->TXDAT = data;

                //Wait until a data is received
                while ((spi_bus->SPI->SR & ((0x01 << 2) | (0x01 << 4))) != 0x04);
                // Get the received data
                data = spi_bus->SPI->RXDAT;

                if (recv_ptr != RT_NULL)
                {
                    *recv_ptr++ = data;
                }
            }
        }
    }

    /* release CS */
    if (message->cs_release)
    {
        LPC_GPIO_PORT->SET[spi_cs->port] |= (0x01 << spi_cs->pin);
    }

    return message->length;
};

/** \brief init and register lpc spi bus.
 *
 * \param SPI: lpc SPI, e.g: LPC_SSP0,LPC_SSP1,LPC_SSP2.
 * \param lpc_spi: lpc spi bus struct.
 * \param spi_bus_name: spi bus name, e.g: "spi1"
 * \return
 *
 */
rt_err_t lpc_spi_register(LPC_SPI0_Type *SPI,
                          struct lpc_spi_bus *lpc_spi,
                          const char *spi_bus_name)
{
    if (SPI == LPC_SPI0)
    {
        lpc_spi->SPI = LPC_SPI0;
       /* Enable the clock for SPI0 */
         LPC_SYSCON->SYSAHBCLKCTRL1 |=  (1UL << 9);
    }
    else if (SPI == LPC_SPI1)
    {
        lpc_spi->SPI = LPC_SPI1;
        /* Enable the clock for SPI1 */
         LPC_SYSCON->SYSAHBCLKCTRL1 |=  (1UL << 10);
    }
    else
    {
        return RT_ENOSYS;
    }

    return rt_spi_bus_register(&lpc_spi->parent, spi_bus_name, &lpc_spi_ops);
}
/* SPI0
SPI0_SCK  <--->PIO0_0
SPI0_MOSI <--->PIO0_16
SPI0_MISO <--->PIO0_10
SPI0_SSEL <--->PIO0_20
*/
int rt_hw_spi_init(void)
{
    /* register spi bus */
    {
        static struct lpc_spi_bus lpc_spi0;
        lpc_spi_register(LPC_SPI0, &lpc_spi0, "spi0");
       	/* Enable the clock for Switch Matrix */
         LPC_SYSCON->SYSAHBCLKCTRL0 |=  (1UL << 12);
	/*
	 * Initialize SPI0 pins connect
	 * SCK0: PINASSIGN3[15:8]: Select P0.0
	 * MOSI0: PINASSIGN3[23:16]: Select P0.16
	 * MISO0: PINASSIGN3[31:24] : Select P0.10
	 * SSEL0: PINASSIGN4[7:0]: Select P0.9
	 */
	LPC_IOCON->PIO0_0 = (0x01<<7);
	LPC_IOCON->PIO0_16 = (0x01<<7);
	LPC_IOCON->PIO0_10 = (0x01<<7);
	LPC_IOCON->PIO0_9 = (0x01<<7);

	LPC_SWM->PINASSIGN3 &= ~(0xff << 8);
	LPC_SWM->PINASSIGN3 |=  (0 << 8);
			
	LPC_SWM->PINASSIGN3 &= ~(0xff << 16);
	LPC_SWM->PINASSIGN3 |=  (16 << 16);
	
  LPC_SWM->PINASSIGN3 &= ~(0xffUL << 24);
	LPC_SWM->PINASSIGN3 |=  (10 << 24);
	
	LPC_SWM->PINASSIGN4 &= ~(0xff << 0);
	LPC_SWM->PINASSIGN4 |=  (9 << 0);


	/* Disable the clock to the Switch Matrix to save power */
	LPC_SYSCON->SYSAHBCLKCTRL0 &=  ~(1UL << 12);
    }
    /* attach cs */
    {
        static struct rt_spi_device spi_device;
        static struct lpc_spi_cs  spi_cs1;
        /* spi10: P4.21 */
       // LPC_IOCON->P4_21 &= ~0x07;
        spi_cs1.port = 0;
        spi_cs1.pin = 9;
       LPC_GPIO_PORT->DIR[spi_cs1.port] |= (0x01 << spi_cs1.pin);
       LPC_GPIO_PORT->SET[spi_cs1.port] |= (0x01 << spi_cs1.pin);
        rt_spi_bus_attach_device(&spi_device, "spi10", "spi1", (void *)&spi_cs1);
    }



    return 0;
}
INIT_BOARD_EXPORT(rt_hw_spi_init);

#endif
