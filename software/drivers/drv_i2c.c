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
* 2013-06-10     xiaonong      The first version for LPC40xx
*/


#include "board.h"
#include "drv_i2c.h"

#ifdef RT_USING_I2C




static void i2c_set_clock(LPC_I2C_TypeDef *I2Cx, uint32_t clock)
{
    uint32_t temp;

    temp = PeripheralClock / clock;

    /* Set the I2C clock value to register */
    I2Cx->SCLH = (uint32_t)(temp / 2);

    I2Cx->SCLL = (uint32_t)(temp - I2Cx->SCLH);
}

static rt_uint32_t i2c_send_addr(LPC_I2C_TypeDef *I2Cx, struct rt_i2c_msg *msg)
{
    rt_uint16_t addr;
    rt_uint16_t flags = msg->flags;
    /* Make sure start bit is not active */
    if (I2Cx->CONSET & I2C_I2CONSET_STA)
    {
        I2Cx->CONCLR = I2C_I2CONCLR_STAC;
    }
    /* Test on the direction to set/reset the read/write bit */
    addr = msg->addr << 1;
    if (flags & RT_I2C_RD)
    {
        /* Set the address bit0 for read */
        addr |= 1;
    }
    I2Cx->CONCLR = I2C_I2CONCLR_SIC;
    /* Send the address */
    I2Cx->DAT = addr & I2C_I2DAT_BITMASK;

    while (!(I2Cx->CONSET & I2C_I2CONSET_SI));

    return (I2Cx->STAT & I2C_STAT_CODE_BITMASK);
}


static rt_size_t lpc_i2c_xfer(struct rt_i2c_bus_device *bus,
                              struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    struct rt_i2c_msg *msg;
    rt_uint32_t i;
    rt_err_t ret = RT_ERROR;
    rt_uint32_t stat = 0;
    struct lpc_i2c_bus *lpc_i2c = (struct lpc_i2c_bus *)bus;
    /*start the i2c bus*/
    stat = lpc_i2c_start(lpc_i2c->I2C);
    if ((I2C_I2STAT_M_TX_RESTART != stat) && (I2C_I2STAT_M_TX_START != stat))
    {
        i2c_dbg("start the i2c bus failed,i2c bus stop!\n");
        goto out;
    }
    for (i = 0; i < num; i++)
    {
        msg = &msgs[i];
        if (!(msg->flags & RT_I2C_NO_START))
        {
            if (i)
            {
                stat = lpc_i2c_start(lpc_i2c->I2C);
                if ((I2C_I2STAT_M_TX_RESTART != stat) && (I2C_I2STAT_M_TX_START != stat))
                {
                    i2c_dbg("restart the i2c bus failed,i2c bus stop!\n");
                    goto out;
                }
            }
            stat = i2c_send_addr(lpc_i2c->I2C, msg);
            if (I2C_I2STAT_M_TX_SLAW_ACK != stat && I2C_I2STAT_M_RX_SLAR_ACK != stat)
            {
                i2c_dbg("send i2c address but no ack,i2c stop!");
                goto out;
            }
        }
        if (msg->flags & RT_I2C_RD)
        {
            ret = lpc_i2c_recv_bytes(lpc_i2c->I2C, msg);
            if (ret >= 1)
                i2c_dbg("read %d byte%s\n",
                        ret, ret == 1 ? "" : "s");
            if (ret < msg->len)
            {
                if (ret >= 0)
                    ret = -RT_EIO;
                goto out;
            }
        }
        else
        {
            ret = lpc_i2c_send_bytes(lpc_i2c->I2C, msg);
            if (ret >= 1)
                i2c_dbg("write %d byte%s\n",
                        ret, ret == 1 ? "" : "s");
            if (ret < msg->len)
            {
                if (ret >= 0)
                    ret = -RT_ERROR;
                goto out;
            }
        }
    }
    ret = i;

out:
    i2c_dbg("send stop condition\n");
    lpc_i2c_stop(lpc_i2c->I2C);

    return ret;
}

/**
 * @brief	Handle I2C0 interrupt by calling I2CM interrupt transfer handler
 * @return	Nothing
 */
void I2C0_IRQHandler(void)
{
	uint32_t status = LPC_I2C0->STAT;
	/* Master Lost Arbitration */
	if (status & I2C_STAT_MSTRARBLOSS) {
		/* Set transfer status as Arbitration Lost */
		xfer->status = I2CM_STATUS_ARBLOST;
		/* Clear Status Flags */
		Chip_I2CM_ClearStatus(pI2C, I2C_STAT_MSTRARBLOSS);
		LPC_I2C0->STAT = I2C_STAT_MSTRARBLOSS ;
	}
	/* Master Start Stop Error */
	else if (status & I2C_STAT_MSTSTSTPERR) {
		/* Set transfer status as Bus Error */
		xfer->status = I2CM_STATUS_BUS_ERROR;
		/* Clear Status Flags */
		LPC_I2C0->STAT = I2C_STAT_MSTSTSTPERR ;
	}
	/* Master is Pending */
	else if (status & I2C_STAT_MSTPENDING) {
		uint32_t mstatus=(LPC_I2C0->STAT & I2C_STAT_MSTSTATE) >> 1;
		/* Branch based on Master State Code */
		switch (mstatus)) {
		/* Master idle */
		case I2C_STAT_MSTCODE_IDLE:
			/* Do Nothing */
			break;

		/* Receive data is available */
		case I2C_STAT_MSTCODE_RXREADY:
			/* Read Data */
			*xfer->rxBuff++ = LPC_I2C0->MSTDAT;
			xfer->rxSz--;
			if (xfer->rxSz) {
				/* Set Continue if there is more data to read */
				LPC_I2C0->MSTCTL = I2C_MSTCTL_MSTCONTINUE;
			}
			else {
				/* Set transfer status as OK */
				xfer->status = I2CM_STATUS_OK;
				/* No data to read send Stop */
				LPC_I2C0->MSTCTL = I2C_MSTCTL_MSTSTOP;
			}
			break;

		/* Master Transmit available */
		case I2C_STAT_MSTCODE_TXREADY:
			if (xfer->txSz) {
				/* If Tx data available transmit data and continue */
				pI2C->MSTDAT = *xfer->txBuff++;
				xfer->txSz--;
				LPC_I2C0->MSTCTL = I2C_MSTCTL_MSTCONTINUE;
			}
			else {
				/* If receive queued after transmit then initiate master receive transfer*/
				if (xfer->rxSz) {
					/* Write Address and RW bit to data register */
					LPC_I2C0->MSTDAT = (xfer->slaveAddr << 1) | 0x1);
					/* Enter to Master Transmitter mode */
					LPC_I2C0->MSTCTL = I2C_MSTCTL_MSTSTART;
				}
				else {
					/* If no receive queued then set transfer status as OK */
					xfer->status = I2CM_STATUS_OK;
					/* Send Stop */
					LPC_I2C0->MSTCTL = I2C_MSTCTL_MSTSTOP;
				}
			}
			break;

		case I2C_STAT_MSTCODE_NACKADR:
			/* Set transfer status as NACK on address */
			xfer->status = I2CM_STATUS_NAK_ADR;
			LPC_I2C0->MSTCTL = I2C_MSTCTL_MSTSTOP;
			break;

		case I2C_STAT_MSTCODE_NACKDAT:
			/* Set transfer status as NACK on data */
			xfer->status = I2CM_STATUS_NAK_DAT;
			LPC_I2C0->MSTCTL = I2C_MSTCTL_MSTSTOP;
			break;

		default:
			/* Default case should not occur*/
			xfer->status = I2CM_STATUS_ERROR;
			break;
		}
	}
	else {
		/* Default case should not occur */
		xfer->status = I2CM_STATUS_ERROR;
	}
}

static const struct rt_i2c_bus_device_ops i2c_ops =
{
    lpc_i2c_xfer,
    RT_NULL,
    RT_NULL
};



void rt_hw_i2c_init(void)
{
    static struct lpc_i2c_bus lpc_i2c1;
    
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 22, IOCON_DIGMODE_EN | I2C_MODE);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 23, IOCON_DIGMODE_EN | I2C_MODE);
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SCL);
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SDA);

    rt_memset((void *)&lpc_i2c1, 0, sizeof(struct lpc_i2c_bus));
    lpc_i2c1.parent.ops = &i2c_ops;
    lpc_i2c_register(LPC_I2C1, &lpc_i2c1, "i2c1");
}
#endif
