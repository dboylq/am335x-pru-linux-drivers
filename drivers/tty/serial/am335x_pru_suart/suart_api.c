/*
 * pru/hal/uart/src/suart_api.c
 *
 * Copyright (C) 2010 Texas Instruments Incorporated
 * Author: Jitendra Kumar <jitendra@mistralsolutions.com>
 * Author: http://www.smartembeddedsystems.com
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as  published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

/*
 *====================
 * Includes
 *====================
 */

#include <linux/platform_device.h>
#include "suart_api.h"
#include "suart_pru_regs.h"
#include "pru.h"
#include "am335x_suart_board.h"
#include "suart_utils.h"
#include "suart_err.h"

#include <linux/mfd/pruss.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/module.h>
#include <linux/io.h>

static unsigned char gUartStatuTable[8];
static arm_pru_iomap pru_arm_iomap;

static s32  suart_set_pru_id(struct device *dev, u32 pru_no);
static void pru_set_rx_tx_mode(struct device *dev, u32 pru_mode, u32 pru_num);
static void pru_set_delay_count(struct device *dev, u32 pru_freq);

void pru_set_ram_data_for (u32 pruNum, arm_pru_iomap * arm_iomap_pru)
{

    PRU_SUART_RegsOvly pru_suart_regs = (PRU_SUART_RegsOvly) arm_iomap_pru->pru_io_addr;
    unsigned char * pu32_pru_ram_base = (unsigned char *) arm_iomap_pru->pru_io_addr;
    pru_suart_tx_cntx_priv * pru_suart_tx_priv = NULL;
    pru_suart_rx_cntx_priv * pru_suart_rx_priv = NULL;
    unsigned int * pu32SrCtlAddr = NULL;
    u32 mcasp_srctl_base = 0;
    u32 mcasp_xbuf_base  = 0;
    u32 mcasp_rbuf_base  = 0;
    u32 mcaspNum = 0;
    u32 chnNum   = 0;
    u32 txSer    = 0;
    u32 rxSer    = 0;
    u32 i;

    if(pruNum == PRUCORE_1) {
        pru_suart_regs = (PRU_SUART_RegsOvly) ((u32) arm_iomap_pru->pru_io_addr + 0x2000);
        pu32_pru_ram_base = (unsigned char *) ((u32) arm_iomap_pru->pru_io_addr + 0x2000);
    }

    for(i = 0; i < NR_SUART; i++) {

        if(pruNum != gSuartInfo[i].pruNum)
            continue;

        mcaspNum = gSuartInfo[i].mcaspNum;
        txSer    = gSuartInfo[i].txSerializer;
        rxSer    = gSuartInfo[i].rxSerializer;

        if(mcaspNum == 0) {
            pu32SrCtlAddr = (u32 *) ((u32) arm_iomap_pru->mcasp_io_addr + 0x180);
            mcasp_srctl_base = MCASP_SRCTL_BASE_ADDR;
            mcasp_xbuf_base  = MCASP_XBUF_BASE_ADDR;
            mcasp_rbuf_base  = MCASP_RBUF_BASE_ADDR;
        }
        else if(mcaspNum == 1) {
            pu32SrCtlAddr = (u32 *) ((u32) arm_iomap_pru->mcasp1_io_addr + 0x180);
            mcasp_srctl_base = MCASP1_SRCTL_BASE_ADDR;
            mcasp_xbuf_base  = MCASP1_XBUF_BASE_ADDR;
            mcasp_rbuf_base  = MCASP1_RBUF_BASE_ADDR;
        }
        else
            return;


        /* Tx context information */
        if(txSer != PRU_SUART_SERIALIZER_NONE) {
            printk(KERN_INFO "%s, uart_num : %d pru_num:%d mcasp_num:%d tx_ser:%d rx_ser:%d\n", __FUNCTION__,
                    gSuartInfo[i].suartNum, pruNum, mcaspNum, txSer, rxSer);
            chnNum = gSuartInfo[i].txPruChn;
            pru_suart_regs[chnNum].CH_Ctrl_Config1.mode   = SUART_CHN_TX;
            pru_suart_regs[chnNum].CH_Ctrl_Config1.asp_id = mcaspNum;
            pru_suart_regs[chnNum].CH_Ctrl_Config1.serializer_num = (0xF & txSer);
            pru_suart_regs[chnNum].CH_Ctrl_Config1.over_sampling  = SUART_DEFAULT_OVRSMPL;
            pru_suart_regs[chnNum].CH_Config2_TXRXStatus.bits_per_char = 8;
            pru_suart_regs[chnNum].CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
            pru_suart_regs[chnNum].Reserved1 = 1;
            *((unsigned int *) (pu32SrCtlAddr + txSer)) =  MCASP_SRCTL_TX_MODE;
            pru_suart_tx_priv = (pru_suart_tx_cntx_priv *) (pu32_pru_ram_base + gSuartInfo[i].ctxBase);
            pru_suart_tx_priv->asp_xsrctl_base = (unsigned int)(mcasp_srctl_base + (txSer << 2));
            pru_suart_tx_priv->asp_xbuf_base = (unsigned int)(mcasp_xbuf_base + (txSer << 2));
            pru_suart_tx_priv->buff_addr = gSuartInfo[i].fmtDataBase;
        }

        /* Rx context information */
        if(rxSer != PRU_SUART_SERIALIZER_NONE) {
            chnNum = gSuartInfo[i].rxPruChn;
            pru_suart_regs[chnNum].CH_Ctrl_Config1.mode = SUART_CHN_RX;
            pru_suart_regs[chnNum].CH_Ctrl_Config1.asp_id = mcaspNum;
            pru_suart_regs[chnNum].CH_Ctrl_Config1.serializer_num = (0xF & rxSer);
            pru_suart_regs[chnNum].CH_Ctrl_Config1.over_sampling = SUART_DEFAULT_OVRSMPL;
            pru_suart_regs[chnNum].CH_Config2_TXRXStatus.bits_per_char = 8;
            pru_suart_regs[chnNum].CH_Config2_TXRXStatus.chn_state = SUART_CHN_ENABLED;
            /* RX is active by default, write the dummy received data at PRU RAM addr 0x1FC to avoid memory corruption */
            pru_suart_regs[chnNum].CH_TXRXData = RX_DEFAULT_DATA_DUMP_ADDR;
            pru_suart_regs[chnNum].Reserved1 = 0;
            *((unsigned int *) (pu32SrCtlAddr + rxSer)) = MCASP_SRCTL_RX_MODE;
            pru_suart_rx_priv = (pru_suart_rx_cntx_priv *) (pu32_pru_ram_base + gSuartInfo[i].ctxBase + 0x10);
            pru_suart_rx_priv->asp_rbuf_base = (unsigned int)(mcasp_rbuf_base + (rxSer << 2));
            pru_suart_rx_priv->asp_rsrctl_base = (unsigned int) (mcasp_srctl_base + (rxSer << 2));
        }
    }

    return;
}

void dump_regs(struct device *dev)
{
#if 0
    unsigned int offset;
    unsigned int value = 0;
    unsigned char *buffer = NULL;
    int i=0, j=0;
    offset = PRU_INTC_STATCLRINT0;
    pruss_readl(dev, offset, &value);
    printk(KERN_INFO "%s, PRU_INTC_STATCLRINT0:0x%08x\n", __FUNCTION__, value);


    offset = 0x24000;
    pruss_readl(dev, offset, &value);
    printk(KERN_INFO "%s, CTRL_REG:0x%08x\n", __FUNCTION__, value);

    offset = 0x24004;
    pruss_readl(dev, offset, &value);
    printk(KERN_INFO "%s, PC:0x%08x\n", __FUNCTION__, value);

#if (!(PRU0_MODE == PRU_MODE_INVALID))
    buffer = (unsigned char *)pru_arm_iomap.pru_io_addr;
#endif
#if (!(PRU1_MODE == PRU_MODE_INVALID))
//    buffer = (unsigned char *)pru_arm_iomap.pru_io_addr+0x2000;
#endif
    buffer += 0x80;
    printk(KERN_INFO "0x%08X    %02x %02x %02x %02x %02x %02x %02x %02x\n",
            (unsigned int)buffer, buffer[i], buffer[i+1], buffer[i+2],buffer[i+3],
            buffer[i+4],buffer[i+5],buffer[i+6],buffer[i+7]);

    buffer += 0x8;
    printk(KERN_INFO "0x%08X    %02x %02x %02x %02x %02x %02x %02x %02x\n",
            (unsigned int)buffer, buffer[i], buffer[i+1], buffer[i+2],buffer[i+3],
            buffer[i+4],buffer[i+5],buffer[i+6],buffer[i+7]);

#if (!(PRU0_MODE == PRU_MODE_INVALID))
    buffer = (unsigned char *)pru_arm_iomap.pru_io_addr;
#endif
#if (!(PRU1_MODE == PRU_MODE_INVALID))
//    buffer = (unsigned char *)pru_arm_iomap.pru_io_addr+0x2000;
#endif

    for(j=0; j<16; j++) {
        printk(KERN_INFO "0x%08X    %02x %02x %02x %02x %02x %02x %02x %02x\n",
                (unsigned int)buffer, buffer[i], buffer[i+1], buffer[i+2],buffer[i+3],
                buffer[i+4],buffer[i+5],buffer[i+6],buffer[i+7]);

        buffer += 0x8;
    }
#if 0
    printk(KERN_INFO "0x%08X    %02x %02x %02x %02x %02x %02x %02x %02x\n",
            (unsigned int)buffer, buffer[i], buffer[i+1], buffer[i+2],buffer[i+3],
            buffer[i+4],buffer[i+5],buffer[i+6],buffer[i+7]);

    buffer += 0x8;
    printk(KERN_INFO "0x%08X    %02x %02x %02x %02x %02x %02x %02x %02x\n",
            (unsigned int)buffer, buffer[i], buffer[i+1], buffer[i+2],buffer[i+3],
            buffer[i+4],buffer[i+5],buffer[i+6],buffer[i+7]);

    buffer += 0x8;
    printk(KERN_INFO "0x%08X    %02x %02x %02x %02x %02x %02x %02x %02x\n",
            (unsigned int)buffer, buffer[i], buffer[i+1], buffer[i+2],buffer[i+3],
            buffer[i+4],buffer[i+5],buffer[i+6],buffer[i+7]);
#endif
#endif
    return;
}
/*
 * suart Initialization routine
 */
short pru_softuart_init(struct device *dev,
        unsigned int txBaudValue,
        unsigned int rxBaudValue,
        unsigned int oversampling,
        unsigned char *pru_suart_emu_code,
        unsigned int fw_size, arm_pru_iomap * arm_iomap_pru)
{
    unsigned int am335x_addr;
    short status = PRU_SUART_SUCCESS;
    short idx;
    short retval;

    if ((PRU0_MODE == PRU_MODE_INVALID) && (PRU1_MODE == PRU_MODE_INVALID)) {
        return PRU_SUART_FAILURE;
    }

    pru_arm_iomap.pru_io_addr = arm_iomap_pru->pru_io_addr;
    pru_arm_iomap.mcasp_io_addr = arm_iomap_pru->mcasp_io_addr;
    pru_arm_iomap.mcasp1_io_addr = arm_iomap_pru->mcasp1_io_addr;
    pru_arm_iomap.pFifoBufferPhysBase = arm_iomap_pru->pFifoBufferPhysBase;
    pru_arm_iomap.pFifoBufferVirtBase = arm_iomap_pru->pFifoBufferVirtBase;
    pru_arm_iomap.pru_clk_freq = arm_iomap_pru->pru_clk_freq;

#ifdef CONFIG_AM33XX_SUART_MCASP0
    am335x_addr = (unsigned int)arm_iomap_pru->mcasp_io_addr;
    /* Configure McASP0  */
    suart_mcasp_config(am335x_addr, txBaudValue, rxBaudValue, oversampling,
            arm_iomap_pru, 0);
    printk(KERN_INFO "Configuring McASP0\n");

	retval = pruss_enable_system_interrupt(dev, 54); // McASP0 sys event
	if (-1 == retval)
		return status;
	retval = pruss_enable_system_interrupt(dev, 55); // McASP0 sys event
	if (-1 == retval)
		return status;
#endif

#ifdef CONFIG_AM33XX_SUART_MCASP1
    am335x_addr = (unsigned int)arm_iomap_pru->mcasp1_io_addr;
    /* Configure McASP0  */
    suart_mcasp_config(am335x_addr, txBaudValue, rxBaudValue, oversampling,
            arm_iomap_pru, 1);
    printk(KERN_INFO "Configuring McASP1\n");

	retval = pruss_enable_system_interrupt(dev, 33); // McASP1 sys event
	if (-1 == retval)
		return status;
	retval = pruss_enable_system_interrupt(dev, 34); // McASP1 sys event
	if (-1 == retval)
		return status;
#endif

#if (!(PRU0_MODE == PRU_MODE_INVALID))
    pruss_enable(dev, PRUSS_NUM0);

    for (idx = 0; idx < (PRU0_DATARAM_SIZE / sizeof(int)); idx++)
        pruss_writel(dev, (PRU0_DATARAM_OFFSET + (idx * sizeof(int))), 0);

    pruss_load(dev, PRUSS_NUM0, (u32 *)pru_suart_emu_code, (fw_size / sizeof(u32)));

	retval = pruss_enable_system_interrupts(dev, PRUSS_NUM0);
	if (-1 == retval)
		return status;

    printk(KERN_INFO "ENABLING PRU0\n");
#endif
#if (!(PRU1_MODE == PRU_MODE_INVALID))
    pruss_enable(dev, PRUSS_NUM1);
    for (idx = 0; idx < (PRU1_DATARAM_SIZE / sizeof(int)); idx++)
        pruss_writel(dev, (PRU1_DATARAM_OFFSET + (idx * sizeof(int))), 0);

    pruss_load(dev, PRUSS_NUM1, (u32 *)pru_suart_emu_code, (fw_size / sizeof(u32)));

	retval = pruss_enable_system_interrupts(dev, PRUSS_NUM1);
	if (-1 == retval)
		return status;

    printk(KERN_INFO "ENABLING PRU1\n");
#endif

    pru_set_delay_count (dev, pru_arm_iomap.pru_clk_freq);

#if (!(PRU0_MODE == PRU_MODE_INVALID))
    suart_set_pru_id(dev, PRUSS_NUM0);
    pru_set_rx_tx_mode(dev, PRU0_MODE, PRUCORE_0);
    printk(KERN_INFO "PROGRAM ID PRU0\n");
    pru_set_ram_data_for (PRUCORE_0, arm_iomap_pru);
#endif

#if (!(PRU1_MODE == PRU_MODE_INVALID))
    suart_set_pru_id(dev, PRUSS_NUM1);
    pru_set_rx_tx_mode(dev, PRU1_MODE, PRUCORE_1);
    printk(KERN_INFO "PROGRAM ID PRU1\n");
    pru_set_ram_data_for (PRUCORE_1, arm_iomap_pru);
#endif

#if (!(PRU0_MODE == PRU_MODE_INVALID))
    pruss_run(dev, PRUSS_NUM0);
    printk(KERN_INFO "RUN PRU0\n");
#endif
#if (!(PRU1_MODE == PRU_MODE_INVALID))
    pruss_run(dev, PRUSS_NUM1);
    printk(KERN_INFO "RUN PRU1\n");
#endif

    dump_regs(dev);
    /* Initialize gUartStatuTable */
    for (idx = 0; idx < 8; idx++) {
        gUartStatuTable[idx] = ePRU_SUART_UART_FREE;
    }

    return status;
}

static short pru_get_chn_num_offset_from_handle(suart_handle hUart, unsigned short *chNum, unsigned int *offset)
{
    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    }

    if (hUart->uartPru == PRUCORE_0) {
        /* PRU0 */
        *offset = PRU_SUART_PRU0_CH0_OFFSET;
    } else {
        /* PRU1 */
        *offset = PRU_SUART_PRU1_CH0_OFFSET;
    }

    *chNum = gSuartInfo[hUart->uartNum-1].txPruChn;

    return PRU_SUART_SUCCESS;
}

static void pru_set_rx_tx_mode(struct device *dev, u32 pru_mode, u32 pru_num)
{
    u32 pru_offset;

    if (pru_num == PRUSS_NUM0)
        pru_offset = PRU_SUART_PRU0_RX_TX_MODE;
    else if (pru_num == PRUSS_NUM1)
        pru_offset = PRU_SUART_PRU1_RX_TX_MODE;
    else
        return;

    pruss_writeb(dev, pru_offset, (u8) pru_mode);
    return;
}

void pru_set_fifo_timeout(struct device *dev, u32 timeout)
{
#if (!(PRU0_MODE == PRU_MODE_INVALID))
    pruss_writew(dev, PRU_SUART_PRU0_IDLE_TIMEOUT_OFFSET, (u16)timeout);
#endif
#if (!(PRU1_MODE == PRU_MODE_INVALID))
    pruss_writew(dev, PRU_SUART_PRU1_IDLE_TIMEOUT_OFFSET, (u16)timeout);
#endif
    return;
}


static void pru_set_delay_count(struct device *dev, u32 pru_freq)
{
    u32 delay_cnt;

    if (pru_freq == PRU_CLK_228)
        delay_cnt = 5;
    else if (pru_freq == PRU_CLK_186)
        delay_cnt = 5;
    else
        delay_cnt = 3;

#if (!(PRU0_MODE == PRU_MODE_INVALID))
    pruss_writeb(dev, PRU_SUART_PRU0_DELAY_OFFSET, (u8) delay_cnt);
#endif
#if (!(PRU1_MODE == PRU_MODE_INVALID))
    pruss_writeb(dev, PRU_SUART_PRU1_DELAY_OFFSET, (u8) delay_cnt);
#endif

    return;
}


void pru_mcasp_deinit (void)
{
#ifdef CONFIG_AM33XX_SUART_MCASP0
    suart_mcasp_reset (&pru_arm_iomap, 0);
#endif
#ifdef CONFIG_AM33XX_SUART_MCASP1
    suart_mcasp_reset (&pru_arm_iomap, 1);
#endif
    return;
}

short pru_softuart_deinit(struct device *dev)
{
    unsigned int offset;
    short s16retval = 0;
    unsigned int u32value = 0;

#if (!(PRU0_MODE == PRU_MODE_INVALID))
    pruss_disable_system_interrupt(dev, 33); // McASP1 sys event
    pruss_disable_system_interrupt(dev, 34); // McASP1 sys event
	pruss_disable_system_interrupts(dev, PRUSS_NUM0);
    pruss_disable(dev, PRUSS_NUM0);
#endif
#if (!(PRU1_MODE == PRU_MODE_INVALID))
    pruss_disable_system_interrupt(dev, 54); // McASP1 sys event
    pruss_disable_system_interrupt(dev, 55); // McASP1 sys event
	pruss_disable_system_interrupts(dev, PRUSS_NUM1);
    pruss_disable(dev, PRUSS_NUM1);
#endif

    offset = PRU_INTC_STATCLRINT1;
    u32value = 0xFFFFFFFF;
    s16retval = pruss_writel(dev, offset, u32value);
    if (-1 == s16retval) {
        return -1;
    }
    offset = PRU_INTC_STATCLRINT0;
    u32value = 0xFFFFFFFF;
    s16retval = pruss_writel(dev, offset, u32value);
    if (-1 == s16retval) {
        return -1;
    }

    return PRU_SUART_SUCCESS;
}

/*
 * suart Instance open routine
 */
short pru_softuart_open(suart_handle hSuart)
{
    short status = PRU_SUART_SUCCESS;

    if((hSuart->uartNum > MAX_SUARTS_SUPPORTED) || (!hSuart)) {
        status = SUART_INVALID_UART_NUM;
    } else {
        hSuart->uartStatus = ePRU_SUART_UART_IN_USE;
        hSuart->uartType = (PRU_SUART_HALF_TX | PRU_SUART_HALF_RX);
        hSuart->uartTxChannel = gSuartInfo[hSuart->uartNum-1].txSerializer;
        hSuart->uartRxChannel = gSuartInfo[hSuart->uartNum-1].rxSerializer;
    }

    return (status);
}

/*
 * suart instance close routine
 */
short pru_softuart_close(suart_handle hUart)
{
    short status = SUART_SUCCESS;

    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    } else {
        gUartStatuTable[hUart->uartNum - 1] = ePRU_SUART_UART_FREE;
        /* Reset the Instance to Invalid */
        hUart->uartNum = PRU_SUART_UARTx_INVALID;
        hUart->uartStatus = ePRU_SUART_UART_FREE;
    }
    return (status);
}

/*
 * suart routine for setting relative baud rate
 */
short pru_softuart_setbaud(struct device *dev, suart_handle hUart,
        unsigned short txClkDivisor, unsigned short rxClkDivisor)
{
    unsigned int offset;
    unsigned int pruOffset;
    short status = SUART_SUCCESS;
    unsigned short chNum;
    unsigned short regval = 0;

    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    }

    /* Set the clock divisor value into the McASP */
    if ((txClkDivisor > 385) || (txClkDivisor == 0)) {
        return SUART_INVALID_CLKDIVISOR;
    }

    if ((rxClkDivisor > 385) || (rxClkDivisor == 0)) {
        return SUART_INVALID_CLKDIVISOR;
    }

    chNum = hUart->uartNum - 1;

    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        if((status = pru_get_chn_num_offset_from_handle(hUart, &chNum, &pruOffset))
                != PRU_SUART_SUCCESS) {
            return status;
        }
    }
    else if (PRU0_MODE == PRU_MODE_TX_ONLY ) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_TX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else {
        return PRU_MODE_INVALID;
    }

    if (txClkDivisor != 0) {
        offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
            PRU_SUART_CH_CONFIG1_OFFSET;
        pruss_readw(dev, offset, &regval);
        regval &= (~0x3FF);
        regval |= txClkDivisor;
        pruss_writew(dev, offset, regval);
    }

    if (PRU0_MODE == PRU_MODE_RX_ONLY ) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        chNum++;
    }
    else {
        return PRU_MODE_INVALID;
    }

    regval = 0;
    if (rxClkDivisor != 0) {
        offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
            PRU_SUART_CH_CONFIG1_OFFSET;
		pruss_readw(dev, offset, &regval);
        regval &= (~0x3FF);
        regval |= txClkDivisor;
		pruss_writew(dev, offset, regval);
    }

    return status;
}

/*
 * suart routine for setting parity for a specific uart
 */
short pru_softuart_setparity (struct device *dev, suart_handle hUart, unsigned short tx_parity, unsigned short rx_parity)
{
    unsigned int offset;
    unsigned int pruOffset;
    short status = SUART_SUCCESS;
    unsigned short chNum;
    unsigned int  reg_val;

    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    }

    /*
     * NOTE:
     * The supported parities are none, odd, and even
     */

    if (tx_parity > ePRU_SUART_PARITY_EVEN || rx_parity > ePRU_SUART_PARITY_EVEN) {
        return PRU_SUART_ERR_PARAMETER_INVALID;
    }

    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        if((status = pru_get_chn_num_offset_from_handle(hUart, &chNum, &pruOffset))
                != PRU_SUART_SUCCESS) {
            return status;
        }
    }
    else if (PRU0_MODE == PRU_MODE_TX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_TX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else {
        return PRU_MODE_INVALID;
    }

    if (tx_parity >= 0) {
        offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
            PRU_SUART_CH_CONFIG2_OFFSET;

        pruss_readb(dev, offset, (u8 *)&reg_val);

        reg_val &= ~(PRU_SUART_CH_CONFIG2_PARITY_MASK);
        reg_val |= (tx_parity << PRU_SUART_CH_CONFIG2_PARITY_SHIFT);

        pruss_writeb(dev, offset, (u8)reg_val);
    }

    if (PRU0_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        chNum++;
    }
    else {
        return PRU_MODE_INVALID;
    }

    if (rx_parity >= 0) {

        offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
            PRU_SUART_CH_CONFIG2_OFFSET;

        pruss_readb(dev, offset, (u8 *)&reg_val);

        reg_val &= ~(PRU_SUART_CH_CONFIG2_PARITY_MASK);
        reg_val |= (rx_parity << PRU_SUART_CH_CONFIG2_PARITY_SHIFT);

        pruss_writeb(dev, offset, (u8)reg_val);
    }

    return (status);
}

/*
 * suart routine for setting number of stop bits for a specific uart
 */
short pru_softuart_setstopbits(struct device *dev, suart_handle hUart, unsigned short stop_bits)
{
    unsigned int offset;
    unsigned int pruOffset;
    short status = SUART_SUCCESS;
    unsigned short chNum;
    unsigned int  reg_val;

    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    }
    /*
     * NOTE:
     * The supported stop bits are 1 and 2
     */

    if (stop_bits > ePRU_SUART_2STOPBITS || stop_bits < ePRU_SUART_1STOPBITS) {
        return PRU_SUART_ERR_PARAMETER_INVALID;
    }

    chNum = hUart->uartNum - 1;

    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        if((status = pru_get_chn_num_offset_from_handle(hUart, &chNum, &pruOffset))
                != PRU_SUART_SUCCESS) {
            return status;
        }
    }
    else if (PRU0_MODE == PRU_MODE_TX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_TX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else {
        return PRU_MODE_INVALID;
    }
    if (stop_bits == 1 || stop_bits == 2) {

        offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
            PRU_SUART_CH_CTRL_OFFSET;

        pruss_readb(dev, offset, (u8 *)&reg_val);

        reg_val &= ~(PRU_SUART_CH_CNTL_STOPBIT_MASK);
        reg_val |= ((stop_bits - 1) << PRU_SUART_CH_CNTL_STOPBIT_SHIFT);

        pruss_writeb(dev, offset, (u8)reg_val);
    }

    return (status);

}


/*
 * suart routine for setting number of bits per character for a specific uart
 */
short pru_softuart_setdatabits (struct device *dev, suart_handle hUart, unsigned short txDataBits,
        unsigned short rxDataBits, unsigned short stopBits)
{
    unsigned int offset;
    unsigned int pruOffset;
    short status = SUART_SUCCESS;
    unsigned short chNum;
    unsigned int  reg_val;
    unsigned int  temp;

    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    }

    /*
     * NOTE:
     * The supported data bits are 6, 7, 8, 9, 10, 11 and 12 bits per character
     */

    if ((txDataBits < ePRU_SUART_DATA_BITS6) || (txDataBits > ePRU_SUART_DATA_BITS12)) {
        return PRU_SUART_ERR_PARAMETER_INVALID;
    }

    if ((rxDataBits < ePRU_SUART_DATA_BITS6) || (rxDataBits > ePRU_SUART_DATA_BITS12)) {
        return PRU_SUART_ERR_PARAMETER_INVALID;
    }

    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH))
    {
        if((status = pru_get_chn_num_offset_from_handle(hUart, &chNum, &pruOffset))
                != PRU_SUART_SUCCESS) {
            return status;
        }
    }
    else if (PRU0_MODE == PRU_MODE_TX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_TX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else {
        return PRU_MODE_INVALID;
    }


    if (txDataBits != 0) {
        offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
            PRU_SUART_CH_CONFIG2_OFFSET;

        pruss_readb(dev, offset, (u8 *)&reg_val);

        temp = (reg_val & PRU_SUART_CH_CONFIG2_PARITY_MASK) >> PRU_SUART_CH_CONFIG2_PARITY_SHIFT;
        /* Check if parity implemented */
        if ((temp == 0x1) | (temp == 0x2))
            txDataBits += 1;

        /* Check if 2 stop bits implemented */
        if(stopBits == 2){
            txDataBits += 1;
        }

        reg_val &= ~(0xF);
        reg_val |= txDataBits;
        pruss_writeb(dev, offset, (u8)reg_val);
    }

    if (PRU0_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        chNum++;
    }
    else {
        return PRU_MODE_INVALID;
    }

    if (rxDataBits != 0) {
        offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
            PRU_SUART_CH_CONFIG2_OFFSET;

        pruss_readb(dev, offset, (u8 *)&reg_val);

        temp = (reg_val & PRU_SUART_CH_CONFIG2_PARITY_MASK) >> PRU_SUART_CH_CONFIG2_PARITY_SHIFT;
        if ((temp == 0x1) | (temp == 0x2))
            rxDataBits += 1;
        reg_val &= ~(0xF);
        reg_val |= rxDataBits;

        pruss_writeb(dev, offset, (u8)reg_val);
    }

    return (status);
}

/*
 * suart routine to configure specific uart
 */
short pru_softuart_setconfig(struct device *dev, suart_handle hUart, suart_config * configUart)
{
    unsigned int offset;
    unsigned int pruOffset;
    short status = SUART_SUCCESS;
    unsigned short chNum;
    unsigned short regVal = 0;

    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    }

    /*
     * NOTE:
     * Dependent baud rate for the given UART, the value MUST BE LESS THAN OR
     * EQUAL TO 64, preScalarValue <= 64
     */
    /* Validate the value of relative buad rate */
    if ((configUart->txClkDivisor > 384) || (configUart->rxClkDivisor > 384)) {
        return SUART_INVALID_CLKDIVISOR;
    }
    /* Validate the bits per character */
    if ((configUart->txBitsPerChar < 8) || (configUart->txBitsPerChar > 14)) {
        return PRU_SUART_ERR_PARAMETER_INVALID;
    }

    if ((configUart->rxBitsPerChar < 8) || (configUart->rxBitsPerChar > 14)) {
        return PRU_SUART_ERR_PARAMETER_INVALID;
    }

    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        if((status = pru_get_chn_num_offset_from_handle(hUart, &chNum, &pruOffset))
                != PRU_SUART_SUCCESS) {
            return status;
        }

    }
    else if (PRU0_MODE == PRU_MODE_TX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_TX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else {
        return PRU_MODE_INVALID;
    }

    /* Configuring the Transmit part of the given UART */
    if (configUart->TXSerializer != PRU_SUART_SERIALIZER_NONE) {
        /* Serializer has been as TX in mcasp config, by writing 1 in bits corresponding to tx serializer
           in PFUNC regsiter i.e. already set to GPIO mode PRU code will set then back to MCASP mode once
           TX request for that serializer is posted. It is required because at this point Mcasp is accessed
           by both PRU and DSP have lower priority for Mcasp in comparison to PRU and DPS keeps on looping
           there only.
         */

        /*
           suart_mcasp_tx_serialzier_set (configUart->TXSerializer, &pru_arm_iomap);
         */

        /* Configuring TX serializer  */
        offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
            PRU_SUART_CH_CTRL_OFFSET;

        pruss_readw(dev, offset, &regVal);
        regVal |= (configUart->TXSerializer << PRU_SUART_CH_CTRL_SR_SHIFT);
        pruss_writew(dev, offset, regVal);

        /* Configuring the Transmit part of the given UART */
        /* Configuring TX prescalar value */
        offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
            PRU_SUART_CH_CONFIG1_OFFSET;

        pruss_readw(dev, offset, &regVal);
        regVal = regVal | (configUart->txClkDivisor << PRU_SUART_CH_CONFIG1_DIVISOR_SHIFT);
        pruss_writew(dev, offset, regVal);

        /* Configuring TX bits per character value */
        offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) + PRU_SUART_CH_CONFIG2_OFFSET;
        pruss_readw(dev, offset, &regVal);
        regVal = regVal | (configUart->txBitsPerChar <<
                PRU_SUART_CH_CONFIG2_BITPERCHAR_SHIFT);
        pruss_writew(dev, offset, regVal);
    }

    if (PRU0_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        chNum++;
    }
    else {
        return PRU_MODE_INVALID;
    }

    /* Configuring the Transmit part of the given UART */
    if (configUart->RXSerializer != PRU_SUART_SERIALIZER_NONE) {
        /* Configuring RX serializer  */
        offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) + PRU_SUART_CH_CTRL_OFFSET;

        pruss_readw(dev, offset, &regVal);
        regVal |=  (configUart->RXSerializer << PRU_SUART_CH_CTRL_SR_SHIFT);
        pruss_writew(dev, offset, regVal);

        /* Configuring RX prescalar value and Oversampling */
        offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) + PRU_SUART_CH_CONFIG1_OFFSET;

        pruss_readw(dev, offset, &regVal);
        regVal = regVal | (configUart->rxClkDivisor << PRU_SUART_CH_CONFIG1_DIVISOR_SHIFT) |
            (configUart->Oversampling << PRU_SUART_CH_CONFIG1_OVS_SHIFT);
        pruss_writew(dev, offset, regVal);

        /* Configuring RX bits per character value */
        offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
            PRU_SUART_CH_CONFIG2_OFFSET;
        pruss_readw(dev, offset, &regVal);
        regVal = regVal | (configUart->rxBitsPerChar << PRU_SUART_CH_CONFIG1_DIVISOR_SHIFT);
        pruss_writew(dev, offset, regVal);
    }

    return (status);
}

/*
 * suart routine for getting the number of bytes transfered
 */
short pru_softuart_getTxDataLen(struct device *dev, suart_handle hUart)
{
    unsigned int offset;
    unsigned int pruOffset;
    unsigned short chNum;
    unsigned short u16ReadValue = 0;
    short status = SUART_SUCCESS;

    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    }

    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        if((status = pru_get_chn_num_offset_from_handle(hUart, &chNum, &pruOffset))
                != PRU_SUART_SUCCESS) {
            return status;
        }
    }
    else if (PRU0_MODE == PRU_MODE_TX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_TX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else  {
        return PRU_MODE_INVALID;
    }

    /* Transmit channel number is (UartNum * 2) - 2  */

    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
        PRU_SUART_CH_CONFIG2_OFFSET;
    pruss_readw(dev, offset, &u16ReadValue);

    u16ReadValue = ((u16ReadValue & PRU_SUART_CH_CONFIG1_DIVISOR_MASK) >>
            PRU_SUART_CH_CONFIG2_DATALEN_SHIFT);
    return (u16ReadValue);
}

/*
 * suart routine for getting the number of bytes received
 */
short pru_softuart_getRxDataLen(struct device *dev, suart_handle hUart)
{
    unsigned int offset;
    unsigned int pruOffset;
    unsigned short chNum;
    unsigned short u16ReadValue = 0;
    short status = SUART_SUCCESS;

    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    }

    chNum = hUart->uartNum - 1;

    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        if((status = pru_get_chn_num_offset_from_handle(hUart, &chNum, &pruOffset))
                != PRU_SUART_SUCCESS) {
            return status;
        }

        chNum++;
    }
    else if (PRU0_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else {
        return PRU_MODE_INVALID;
    }

    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) + PRU_SUART_CH_CONFIG2_OFFSET;
    pruss_readw(dev, offset, &u16ReadValue);

    u16ReadValue = ((u16ReadValue & PRU_SUART_CH_CONFIG1_DIVISOR_MASK) >>
            PRU_SUART_CH_CONFIG2_DATALEN_SHIFT);

    return (u16ReadValue);
}

/*
 * suart routine to get the configuration information from a specific uart
 */
short pru_softuart_getconfig(struct device *dev, suart_handle hUart, suart_config * configUart)
{
    unsigned int offset;
    unsigned int pruOffset;
    unsigned short chNum;
    unsigned short regVal = 0;
    short status = SUART_SUCCESS;

    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    }

    /*
     * NOTE:
     * Dependent baud rate for the given UART, the value MUST BE LESS THAN OR
     * EQUAL TO 64, preScalarValue <= 64
     */

    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        if((status = pru_get_chn_num_offset_from_handle(hUart, &chNum, &pruOffset))
                != PRU_SUART_SUCCESS) {
            return status;
        }
    }
    else if (PRU0_MODE == PRU_MODE_TX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_TX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else  {
        return PRU_MODE_INVALID;
    }

    /* Configuring the Transmit part of the given UART */
    /* Configuring TX serializer  */
    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) + PRU_SUART_CH_CTRL_OFFSET;
    pruss_readw(dev, offset, &regVal);
    configUart->TXSerializer = ((regVal & PRU_SUART_CH_CTRL_SR_MASK) >>
            PRU_SUART_CH_CTRL_SR_SHIFT);

    /* Configuring TX prescalar value */
    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) + PRU_SUART_CH_CONFIG1_OFFSET;
    pruss_readw(dev, offset, &regVal);
    configUart->txClkDivisor = ((regVal & PRU_SUART_CH_CONFIG1_DIVISOR_MASK) >>
            PRU_SUART_CH_CONFIG1_DIVISOR_SHIFT);

    /* Configuring TX bits per character value */
    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) + PRU_SUART_CH_CONFIG2_OFFSET;
    pruss_readw(dev, offset, &regVal);
    configUart->txBitsPerChar = ((regVal & PRU_SUART_CH_CONFIG1_DIVISOR_MASK) >>
            PRU_SUART_CH_CONFIG1_DIVISOR_SHIFT);

    if (PRU0_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        chNum++;
    }
    else {
        return PRU_MODE_INVALID;
    }

    /* Configuring the Transmit part of the given UART */
    /* Configuring RX serializer  */
    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) + PRU_SUART_CH_CTRL_OFFSET;
    pruss_readw(dev, offset, &regVal);
    configUart->RXSerializer = ((regVal & PRU_SUART_CH_CTRL_SR_MASK) >>
         PRU_SUART_CH_CTRL_SR_SHIFT);

    /* Configuring RX prescalar value and Oversampling */
    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) + PRU_SUART_CH_CONFIG1_OFFSET;
    pruss_readw(dev, offset, &regVal);
    configUart->rxClkDivisor = ((regVal & PRU_SUART_CH_CONFIG1_DIVISOR_MASK) >>
         PRU_SUART_CH_CONFIG1_DIVISOR_SHIFT);
    configUart->Oversampling = ((regVal & PRU_SUART_CH_CONFIG1_OVS_MASK) >>
         PRU_SUART_CH_CONFIG1_OVS_SHIFT);

    /* Configuring RX bits per character value */
    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) + PRU_SUART_CH_CONFIG2_OFFSET;
    pruss_readw(dev, offset, &regVal);
    configUart->rxBitsPerChar = ((regVal & PRU_SUART_CH_CONFIG1_DIVISOR_MASK) >>
         PRU_SUART_CH_CONFIG1_DIVISOR_SHIFT);

    return (status);
}



/*
 * suart data transmit routine
 */
short pru_softuart_write (struct device *dev, suart_handle hUart, unsigned int *ptTxDataBuf, unsigned short dataLen)
{
    unsigned int offset = 0;
    unsigned int pruOffset;
    short status = SUART_SUCCESS;
    unsigned short chNum;
    unsigned short regVal = 0;

    unsigned short pru_num;

//    dump_regs(dev);

    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    }

    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        if((status = pru_get_chn_num_offset_from_handle(hUart, &chNum, &pruOffset))
                != PRU_SUART_SUCCESS) {
            return status;
        }

        pru_num = hUart->uartPru;
    }
    else if (PRU0_MODE == PRU_MODE_TX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
        pru_num = 0;
    }
    else if (PRU1_MODE == PRU_MODE_TX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
        pru_num = 1;
    }
    else {
        return PRU_MODE_INVALID;
    }

    /* Writing data length to SUART channel register */
    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) + PRU_SUART_CH_CONFIG2_OFFSET;
    pruss_readw(dev, offset, &regVal);
    regVal &= ~PRU_SUART_CH_CONFIG2_DATALEN_MASK;
    regVal = regVal | (dataLen << PRU_SUART_CH_CONFIG2_DATALEN_SHIFT);
    pruss_writew(dev, offset, regVal);

    /* Writing the data pointer to channel TX data pointer */
    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) + PRU_SUART_CH_TXRXDATA_OFFSET;
    pruss_writel(dev, offset, *ptTxDataBuf);

    /* Service Request to PRU */
    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) + PRU_SUART_CH_CTRL_OFFSET;
    pruss_readw(dev, offset, &regVal);

    regVal &= ~(PRU_SUART_CH_CTRL_MODE_MASK |PRU_SUART_CH_CTRL_SREQ_MASK);

    regVal |= (PRU_SUART_CH_CTRL_TX_MODE << PRU_SUART_CH_CTRL_MODE_SHIFT) |
        (PRU_SUART_CH_CTRL_SREQ    <<    PRU_SUART_CH_CTRL_SREQ_SHIFT);

    pruss_writew(dev, offset, regVal);

    /* generate ARM->PRU event */
    pruss_arm_to_pru_intr_set(dev, pru_num);

    return (status);
}

/*
 * suart data receive routine
 */
short pru_softuart_read (struct device *dev, suart_handle hUart, unsigned int *ptDataBuf, unsigned short dataLen)
{
    unsigned int offset = 0;
    unsigned int pruOffset;
    short status = SUART_SUCCESS;
    unsigned short chNum;
    unsigned short regVal = 0;
    unsigned short pru_num;
    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    }

    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        if((status = pru_get_chn_num_offset_from_handle(hUart, &chNum, &pruOffset))
                != PRU_SUART_SUCCESS) {
            return status;
        }

        pru_num = hUart->uartPru;
        chNum++;
    }
    else if (PRU0_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
        pru_num = 0;
    }
    else if (PRU1_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
        pru_num = 1;
    }
    else {
        return PRU_MODE_INVALID;
    }

    /* Writing data length to SUART channel register */
    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) + PRU_SUART_CH_CONFIG2_OFFSET;
    pruss_readw(dev, offset, &regVal);
    regVal &= ~PRU_SUART_CH_CONFIG2_DATALEN_MASK;
    regVal = regVal | (dataLen << PRU_SUART_CH_CONFIG2_DATALEN_SHIFT);
    pruss_writew(dev, offset, regVal);

    /* Writing the data pointer to channel RX data pointer */
    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
        PRU_SUART_CH_TXRXDATA_OFFSET;
    pruss_writel(dev, offset, *ptDataBuf);

    /* Service Request to PRU */
    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) + PRU_SUART_CH_CTRL_OFFSET;
    pruss_readw(dev, offset, &regVal);

    regVal &= ~(PRU_SUART_CH_CTRL_MODE_MASK |PRU_SUART_CH_CTRL_SREQ_MASK);

    regVal |=  ( PRU_SUART_CH_CTRL_RX_MODE << PRU_SUART_CH_CTRL_MODE_SHIFT) |
        (PRU_SUART_CH_CTRL_SREQ << PRU_SUART_CH_CTRL_SREQ_SHIFT);

    pruss_writew(dev, offset, regVal);

    /* enable the timeout interrupt */
    suart_intr_setmask (dev, hUart->uartNum, PRU_RX_INTR, CHN_TXRX_IE_MASK_TIMEOUT);

    /* generate ARM->PRU event */
    pruss_arm_to_pru_intr_set(dev, pru_num);

    return (status);
}

/*
 * suart routine to read the data from the RX FIFO
 */
short pru_softuart_read_data (struct device *dev, suart_handle hUart, Uint8 * pDataBuffer,
        Int32 s32MaxLen, Uint32 * pu32DataRead, unsigned short u16Status,
        Uint32 u32DataRead, Uint32 u32DataLen, Uint8 * pu8SrcAddr,
        unsigned short u16ctrlReg)
{
    short retVal = PRU_SUART_SUCCESS;
    Uint32	u32CharLen = 0;
    Uint32  u32Parity = 0;
    unsigned int offset = 0;
    unsigned int pruOffset;
    unsigned short chNum;

    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    }

    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        if((retVal = pru_get_chn_num_offset_from_handle(hUart, &chNum, &pruOffset))
                != PRU_SUART_SUCCESS) {
            return retVal;
        }

        chNum++;
    }
    else if (PRU0_MODE == PRU_MODE_RX_ONLY ) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else {
        return PRU_MODE_INVALID;
    }

    /* read the character length */
    u32CharLen = u32DataLen & PRU_SUART_CH_CONFIG2_BITPERCHAR_MASK;

    /* read the parity value */
    u32Parity = u32DataLen & PRU_SUART_CH_CONFIG2_PARITY_MASK;

    /* Check if parity is enabled.  If set, subtract u32CharLen - 3.
       Else, subtract u32CharLen - 2.  */
    if( u32Parity > 0 ) {
        u32CharLen -= 3; /* remove the START, PARITY, & STOP bit */
    }
    else {
        u32CharLen -= 2; /* remove the START & STOP bit */
    }

    u32DataLen &= PRU_SUART_CH_CONFIG2_DATALEN_MASK;
    u32DataLen = u32DataLen >> PRU_SUART_CH_CONFIG2_DATALEN_SHIFT;
    u32DataLen ++;

    /* if the character length is greater than 8, then the size doubles */
    if (u32CharLen > 8) {
        u32DataLen *= 2;
    }

    /* Check if the time-out had occured. If, yes, then we need to find the
     * number of bytes read from PRU. Else, we need to read the requested bytes
     */

    if (u16Status & (CHN_TXRX_STATUS_TIMEOUT | CHN_TXRX_STATUS_OVRNERR |
                CHN_TXRX_STATUS_PE | CHN_TXRX_STATUS_FE | CHN_TXRX_STATUS_BI)) {
        /* if the character length is greater than 8, then the size doubles */
        if (u32CharLen > 8) {
            u32DataRead *= 2;
        }

        /* the data corresponding is loaded in second half during the timeout */
        if (u32DataRead > u32DataLen) {
            u32DataRead -= u32DataLen;
            pu8SrcAddr +=  u32DataLen;
        }

        pru_softuart_clrRxFifo (dev, hUart, (unsigned short) u32DataRead, u16ctrlReg);
    }
    else {
        u32DataRead = u32DataLen;
        /* Determine the buffer index by reading the FIFO_OddEven flag*/
        if (u16Status & CHN_TXRX_STATUS_CMPLT) {
            /* if the bit is set, the data is in the first half of the FIFO else
             * the data is in the second half */
            pu8SrcAddr += u32DataLen;
        }
    }

    /* we should be copying only max len given by the application */
    if (u32DataRead > s32MaxLen) {
        u32DataRead = s32MaxLen;
    }

    /* evaluate the virtual address of the FIFO address based on the physical addr */
    pu8SrcAddr = (Uint8 *) ((Uint32) pu8SrcAddr - (Uint32) pru_arm_iomap.pFifoBufferPhysBase +
            (Uint32) pru_arm_iomap.pFifoBufferVirtBase);

    /* Now we have both the data length and the source address. copy */
    for (offset = 0; offset < u32DataRead; offset++) {
        * pDataBuffer++ = * pu8SrcAddr++;
    }
    * pu32DataRead = u32DataRead;

    retVal = PRU_SUART_SUCCESS;

    return (retVal);
}


/*
 * suart routine to get the tx status for a specific uart
 */
short pru_softuart_getTxStatus(struct device *dev, suart_handle hUart)
{
    unsigned int offset;
    unsigned int pruOffset;
    unsigned short status = SUART_SUCCESS;
    unsigned short chNum;

    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    }

    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        if((status = pru_get_chn_num_offset_from_handle(hUart, &chNum, &pruOffset))
                != PRU_SUART_SUCCESS) {
            return status;
        }
    }
    else if (PRU0_MODE == PRU_MODE_TX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_TX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else {
        return PRU_MODE_INVALID;
    }

    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) + PRU_SUART_CH_TXRXSTATUS_OFFSET;
    pruss_readb(dev, offset, (u8 *)&status);
    return (status);
}

short pru_softuart_clrTxStatus(struct device *dev, suart_handle hUart)
{
    unsigned int offset;
    unsigned int pruOffset;
    unsigned short status = SUART_SUCCESS;
    unsigned short chNum;

    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    }

    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        if((status = pru_get_chn_num_offset_from_handle(hUart, &chNum, &pruOffset))
                != PRU_SUART_SUCCESS) {
            return status;
        }
    }
    else if (PRU0_MODE == PRU_MODE_TX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_TX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else {
        return PRU_MODE_INVALID;
    }

    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) + PRU_SUART_CH_TXRXSTATUS_OFFSET;
    pruss_readb(dev, offset, (u8 *)&status);

    status &= ~(0x2);
    pruss_writeb(dev, offset, (u8)status);
    return (status);
}

/*
 * suart routine to get the rx status for a specific uart
 */
short pru_softuart_getRxStatus(struct device *dev, suart_handle hUart)
{
    unsigned int offset;
    unsigned int pruOffset;
    unsigned short status = SUART_SUCCESS;
    unsigned short chNum;

    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    }

    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        if((status = pru_get_chn_num_offset_from_handle(hUart, &chNum, &pruOffset))
                != PRU_SUART_SUCCESS) {
            return status;
        }
        chNum++;
    }
    else if (PRU0_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else {
        return PRU_MODE_INVALID;
    }

    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) + PRU_SUART_CH_TXRXSTATUS_OFFSET;
    pruss_readb(dev, offset, (u8 *)&status);
    return (status);
}

/*
 * suart routine to get the Config2 register for a specific uart
 */
int pru_softuart_getRxFifoBytes(struct device *dev, suart_handle hUart)
{
    unsigned int offset;
    unsigned int pruOffset;
    Uint32  u32DataRead = 0;
    unsigned short chNum;
    unsigned short status = SUART_SUCCESS;

    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    }

    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        if((status = pru_get_chn_num_offset_from_handle(hUart, &chNum, &pruOffset))
                != PRU_SUART_SUCCESS) {
            return status;
        }
        chNum++;
    }
    else if (PRU0_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else {
        return PRU_MODE_INVALID;
    }

    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) + PRU_SUART_CH_BYTESDONECNTR_OFFSET;
    pruss_readb(dev, offset, (u8 *)&u32DataRead);

    return (u32DataRead);
}

/*
 * suart routine to get the TXRX data pointer for a specific uart
 */
int pru_softuart_getRxDataPointer(struct device *dev, suart_handle hUart)
{
    unsigned int offset;
    unsigned int pruOffset;
    Uint8 * pu8SrcAddr = NULL;
    unsigned short chNum;
    unsigned short status = SUART_SUCCESS;

    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    }

    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        if((status = pru_get_chn_num_offset_from_handle(hUart, &chNum, &pruOffset))
                != PRU_SUART_SUCCESS) {
            return status;
        }
        chNum++;
    }
    else if (PRU0_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else {
        return PRU_MODE_INVALID;
    }

    /* Get the data pointer from channel RX data pointer */
    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) + PRU_SUART_CH_TXRXDATA_OFFSET;
    pruss_readl(dev, offset, (u32 *)&pu8SrcAddr);

    return ((int)pu8SrcAddr);
}

/*
 * suart routine to get the number of bytes in the FIFO for a specific uart
 */
int pru_softuart_getRxConfig2(struct device *dev, suart_handle hUart)
{
    unsigned int offset;
    unsigned int pruOffset;
    Uint32  u32DataLen = 0;
    unsigned short chNum;
    unsigned short status = SUART_SUCCESS;

    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    }

    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        if((status = pru_get_chn_num_offset_from_handle(hUart, &chNum, &pruOffset))
                != PRU_SUART_SUCCESS) {
            return status;
        }
        chNum++;
    }
    else if (PRU0_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else {
        return PRU_MODE_INVALID;
    }

    /* Reading data length from SUART channel register */
    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) + PRU_SUART_CH_CONFIG2_OFFSET;
    pruss_readw(dev, offset, (u16 *)&u32DataLen);

    return (u32DataLen);
}

int pru_softuart_getRxCntrlReg(struct device *dev, suart_handle hUart)
{
    unsigned int offset;
    unsigned int pruOffset;
    unsigned short regVal;
    unsigned short chNum;
    unsigned short status = SUART_SUCCESS;

    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    }

    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        if((status = pru_get_chn_num_offset_from_handle(hUart, &chNum, &pruOffset))
                != PRU_SUART_SUCCESS) {
            return status;
        }
        chNum++;
    }
    else if (PRU0_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else {
        return PRU_MODE_INVALID;
    }

    /* Reading data length from SUART channel register */
    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
        PRU_SUART_CH_CTRL_OFFSET;

    pruss_readw(dev, offset, &regVal);

    return (regVal);
}

short pru_softuart_clrRxFifo(struct device *dev, suart_handle hUart, unsigned short regVal, unsigned short cntrlReg)
{
    unsigned int offset;
    unsigned int pruOffset;
    unsigned short status = SUART_SUCCESS;
    unsigned short chNum;
    //	unsigned short regVal;
    unsigned short uartNum;

    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    }

    uartNum = hUart->uartNum;

    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        if((status = pru_get_chn_num_offset_from_handle(hUart, &chNum, &pruOffset))
                != PRU_SUART_SUCCESS) {
            return status;
        }
        chNum++;
    }
    else if (PRU0_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
        uartNum = 0;
    }
    else if (PRU1_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
        uartNum = 1;
    }
    else {
        return PRU_MODE_INVALID;
    }

    /* Reset the number of bytes read into the FIFO */
    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
        PRU_SUART_CH_BYTESDONECNTR_OFFSET;

    //        pru_ram_read_data(offset, (Uint8 *) & regVal, 1, &pru_arm_iomap);
    regVal &= 0x00;

    pruss_writeb(dev, offset, regVal);

    /* Service Request to PRU */
    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
        PRU_SUART_CH_CTRL_OFFSET;

    //	pru_ram_read_data(offset, (Uint8 *) & regVal, 2, &pru_arm_iomap);

    cntrlReg &= ~(PRU_SUART_CH_CTRL_MODE_MASK |PRU_SUART_CH_CTRL_SREQ_MASK);

    cntrlReg |=  ( PRU_SUART_CH_CTRL_RX_MODE << PRU_SUART_CH_CTRL_MODE_SHIFT) |
        (PRU_SUART_CH_CTRL_SREQ << PRU_SUART_CH_CTRL_SREQ_SHIFT);

    pruss_writew(dev, offset, cntrlReg);
    suart_intr_setmask (dev, hUart->uartNum, PRU_RX_INTR, CHN_TXRX_IE_MASK_TIMEOUT);

    /* generate ARM->PRU event */
    pruss_arm_to_pru_intr_set(dev, hUart->uartPru);

    return (status);
}


short pru_softuart_clrRxStatus(struct device *dev, suart_handle hUart)
{
    unsigned int offset;
    unsigned int pruOffset;
    unsigned short status = SUART_SUCCESS;
    unsigned short chNum;

    if (hUart == NULL) {
        return (PRU_SUART_ERR_HANDLE_INVALID);
    }

    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {
        if((status = pru_get_chn_num_offset_from_handle(hUart, &chNum, &pruOffset))
                != PRU_SUART_SUCCESS) {
            return status;
        }

        chNum++;
    }
    else if (PRU0_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
    }
    else if (PRU1_MODE == PRU_MODE_RX_ONLY) {
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else  {
        return PRU_MODE_INVALID;
    }

    offset = pruOffset + (chNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
        PRU_SUART_CH_TXRXSTATUS_OFFSET;
    pruss_readb(dev, offset, (u8 *)&status);

    status &= ~(0xBC);
    pruss_writeb(dev, offset, (u8)status);
    return (status);
}

/*
 * suart_intr_status_read: Gets the Global Interrupt status register
 * for the specified SUART.
 * uartNum < 1 to 6 >
 * txrxFlag < Indicates TX or RX interrupt for the uart >
 */
short pru_softuart_get_isrstatus(struct device *dev, unsigned short uartNum, unsigned short *txrxFlag)
{
    /* initialize the status & Flag to known value */
    *txrxFlag = 0;
    /* Check For Tansmit interrupt */
    if (pruss_system_interrupt_is_set(dev, gSuartInfo[uartNum-1].txSysEvent)) {
        *txrxFlag |= PRU_TX_INTR;
        pruss_system_interrupt_clr(dev, gSuartInfo[uartNum-1].txSysEvent);
    }

    /* Check For Receive interrupt */
    if (pruss_system_interrupt_is_set(dev, gSuartInfo[uartNum-1].rxSysEvent)) {
        *txrxFlag |= PRU_RX_INTR;
        pruss_system_interrupt_clr(dev, gSuartInfo[uartNum-1].rxSysEvent);
    }

    return 0;
}

int pru_intr_clr_isrstatus(struct device *dev, unsigned short uartNum, unsigned int txrxmode)
{
    unsigned int offset     = 0;
    unsigned short txrxFlag = 0;

    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {

        if(gSuartInfo[uartNum-1].pruNum == PRUCORE_0)
            offset = PRU_SUART_PRU0_ISR_OFFSET + 1;
        else if (gSuartInfo[uartNum-1].pruNum == PRUCORE_1)
            offset = PRU_SUART_PRU1_ISR_OFFSET + 1;

    }
    else if (PRU0_MODE == txrxmode) {
        offset = PRU_SUART_PRU0_ISR_OFFSET + 1;
    }
    else if (PRU1_MODE == txrxmode) {
        offset = PRU_SUART_PRU1_ISR_OFFSET + 1;
    }
    else  {
        return PRU_MODE_INVALID;
    }

    pruss_readb(dev, offset, (u8 *)&txrxFlag);
    txrxFlag &= ~(0x2);
    pruss_writeb(dev, offset, (u8)txrxFlag);

    return 0;
}

int suart_pru_to_host_intr_enable (struct device *dev, unsigned short uartNum,
        unsigned int txrxmode, int s32Flag)
{

    u32   intrNum = 0;
    s32   retVal = 0;

    if(txrxmode == PRU_TX_INTR)
        intrNum = gSuartInfo[uartNum-1].txSysEvent;
    else
        intrNum = gSuartInfo[uartNum-1].rxSysEvent;

    if (TRUE == s32Flag)
        retVal = pruss_enable_system_interrupt(dev, intrNum);
    else
        retVal = pruss_disable_system_interrupt(dev, intrNum);

    return (retVal);
}

int suart_intr_setmask(struct device *dev, unsigned short uartNum,
        unsigned int txrxmode, unsigned int intrmask)
{
    unsigned int offset    = 0;
    unsigned int pruOffset = 0;
    unsigned int txrxFlag  = 0;
    unsigned short regval  = 0;
    unsigned int chnNum    = 0;

    chnNum = uartNum -1;
    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {

        if(gSuartInfo[uartNum-1].pruNum == PRUCORE_0) {
            pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
            offset = PRU_SUART_PRU0_IMR_OFFSET;
        }
        else if (gSuartInfo[uartNum-1].pruNum == PRUCORE_1) {
            offset = PRU_SUART_PRU1_IMR_OFFSET;
            pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
        }

        if(txrxmode == PRU_TX_INTR)
            chnNum = gSuartInfo[uartNum-1].txPruChn;
        else
            chnNum = gSuartInfo[uartNum-1].rxPruChn;

    }
    else if (PRU0_MODE == txrxmode) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
        offset = PRU_SUART_PRU0_IMR_OFFSET;
    }
    else if (PRU1_MODE == txrxmode) {
        offset = PRU_SUART_PRU1_IMR_OFFSET;
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else  {
        return PRU_MODE_INVALID;
    }


    regval = 1 << chnNum;

    if (CHN_TXRX_IE_MASK_CMPLT == (intrmask & CHN_TXRX_IE_MASK_CMPLT)) {
        pruss_readw(dev, offset, (u16 *)&txrxFlag);
        txrxFlag &= ~(regval);
        txrxFlag |= regval;
        pruss_writew(dev, offset, txrxFlag);
    }

    if ((intrmask & SUART_GBL_INTR_ERR_MASK) == SUART_GBL_INTR_ERR_MASK) {
        regval = 0;
        pruss_readw(dev, offset, &regval);
        regval &= ~(SUART_GBL_INTR_ERR_MASK);
        regval |= (SUART_GBL_INTR_ERR_MASK);
        pruss_writew(dev, offset, regval);

    }
    /* Framing Error Interrupt Masked */
    if ((intrmask & CHN_TXRX_IE_MASK_FE) == CHN_TXRX_IE_MASK_FE) {
        regval = 0;
        offset = pruOffset + (chnNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
            PRU_SUART_CH_CONFIG1_OFFSET;
        pruss_readw(dev, offset, &regval);
        regval &= ~(CHN_TXRX_IE_MASK_FE);
        regval |= CHN_TXRX_IE_MASK_FE;
        pruss_writew(dev, offset, regval);
    }
    /* Break Indicator Interrupt Masked */
    if (CHN_TXRX_IE_MASK_BI == (intrmask & CHN_TXRX_IE_MASK_BI)) {
        regval = 0;
        offset = pruOffset + (chnNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
            PRU_SUART_CH_CONFIG1_OFFSET;

        pruss_readw(dev, offset, &regval);
        regval &= ~(CHN_TXRX_IE_MASK_BI);
        regval |= CHN_TXRX_IE_MASK_BI;
        pruss_writew(dev, offset, regval);
    }
    /* Timeout error Interrupt Masked */
    if (CHN_TXRX_IE_MASK_TIMEOUT == (intrmask & CHN_TXRX_IE_MASK_TIMEOUT)) {
        regval = 0;
        offset = pruOffset + (chnNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
            PRU_SUART_CH_CONFIG1_OFFSET;

        pruss_readw(dev, offset, &regval);
        regval &= ~(CHN_TXRX_IE_MASK_TIMEOUT);
        regval |= CHN_TXRX_IE_MASK_TIMEOUT;
        pruss_writew(dev, offset, regval);
    }

    /* Overrun error Interrupt Masked */
    if (CHN_RX_IE_MASK_OVRN == (intrmask & CHN_RX_IE_MASK_OVRN)) {
        regval = 0;
        offset = pruOffset + (chnNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
            PRU_SUART_CH_CONFIG1_OFFSET;

        pruss_readw(dev, offset, &regval);
        regval &= ~(CHN_RX_IE_MASK_OVRN);
        regval |= CHN_RX_IE_MASK_OVRN;
        pruss_writew(dev, offset, regval);
    }

    /* Parity error Interrupt Masked */
    if (CHN_TXRX_IE_MASK_PE == (intrmask & CHN_TXRX_IE_MASK_PE)) {
        regval = 0;
        offset = pruOffset + (chnNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
            PRU_SUART_CH_TXRXSTATUS_OFFSET;

        pruss_readw(dev, offset, &regval);
        regval &= ~(CHN_TXRX_IE_MASK_PE);
        regval |= CHN_TXRX_IE_MASK_PE;
        pruss_writew(dev, offset, regval);
    }
    return 0;
}

int suart_intr_clrmask(struct device *dev, unsigned short uartNum,
        unsigned int txrxmode, unsigned int intrmask)
{
    unsigned int offset     = 0;
    unsigned int pruOffset  = 0;
    unsigned short txrxFlag = 0;
    unsigned short regval   = 0;
    unsigned short chnNum   = 0;

    chnNum = uartNum -1;
    if ((PRU0_MODE == PRU_MODE_RX_TX_BOTH) || (PRU1_MODE == PRU_MODE_RX_TX_BOTH)) {

        if(gSuartInfo[uartNum-1].pruNum == PRUCORE_0) {
            pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
            offset = PRU_SUART_PRU0_IMR_OFFSET;
        }
        else if (gSuartInfo[uartNum-1].pruNum == PRUCORE_1) {
            offset = PRU_SUART_PRU1_IMR_OFFSET;
            pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
        }

        if(txrxmode == PRU_TX_INTR)
            chnNum = gSuartInfo[uartNum-1].txPruChn;
        else
            chnNum = gSuartInfo[uartNum-1].rxPruChn;

    }
    else if (PRU0_MODE == txrxmode) {
        pruOffset = PRU_SUART_PRU0_CH0_OFFSET;
        offset = PRU_SUART_PRU0_IMR_OFFSET;
    }
    else if (PRU1_MODE == txrxmode) {
        offset = PRU_SUART_PRU1_IMR_OFFSET;
        pruOffset = PRU_SUART_PRU1_CH0_OFFSET;
    }
    else  {
        return PRU_MODE_INVALID;
    }

    regval = 1 << chnNum;

    if (CHN_TXRX_IE_MASK_CMPLT == (intrmask & CHN_TXRX_IE_MASK_CMPLT)) {
        pruss_readw(dev, offset, &txrxFlag);
        txrxFlag &= ~(regval);
        pruss_writew(dev, offset, txrxFlag);
    }

    if ((intrmask & SUART_GBL_INTR_ERR_MASK) == SUART_GBL_INTR_ERR_MASK) {
        regval = 0;
        pruss_readw(dev, offset, &regval);
        regval &= ~(SUART_GBL_INTR_ERR_MASK);
        pruss_writew(dev, offset, regval);

    }
    /* Framing Error Interrupt Masked */
    if ((intrmask & CHN_TXRX_IE_MASK_FE) == CHN_TXRX_IE_MASK_FE) {
        regval = 0;
        offset = pruOffset + (chnNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
            PRU_SUART_CH_CONFIG1_OFFSET;
        pruss_readw(dev, offset, &regval);
        regval &= ~(CHN_TXRX_IE_MASK_FE);
        pruss_writew(dev, offset, regval);
    }
    /* Break Indicator Interrupt Masked */
    if (CHN_TXRX_IE_MASK_BI == (intrmask & CHN_TXRX_IE_MASK_BI)) {
        regval = 0;
        offset = pruOffset + (chnNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
            PRU_SUART_CH_CONFIG1_OFFSET;

        pruss_readw(dev, offset, &regval);
        regval &= ~(CHN_TXRX_IE_MASK_BI);
        pruss_writew(dev, offset, regval);
    }

    /* Timeout error Interrupt Masked */
    if (CHN_TXRX_IE_MASK_TIMEOUT == (intrmask & CHN_TXRX_IE_MASK_TIMEOUT)) {
        regval = 0;
        offset = pruOffset + (chnNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
            PRU_SUART_CH_CONFIG1_OFFSET;

        pruss_readw(dev, offset, &regval);
        regval &= ~(CHN_TXRX_IE_MASK_TIMEOUT);
        pruss_writew(dev, offset, regval);
    }

    /* Overrun error Interrupt Masked */
    if (CHN_RX_IE_MASK_OVRN == (intrmask & CHN_RX_IE_MASK_OVRN)) {
        regval = 0;
        offset = pruOffset + (chnNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
            PRU_SUART_CH_CONFIG1_OFFSET;

        pruss_readw(dev, offset, &regval);
        regval &= ~(CHN_RX_IE_MASK_OVRN);
        pruss_writew(dev, offset, regval);
    }

    /* Parity error Interrupt Masked */
    if (CHN_TXRX_IE_MASK_PE == (intrmask & CHN_TXRX_IE_MASK_PE)) {
        regval = 0;
        offset = pruOffset + (chnNum * SUART_NUM_OF_BYTES_PER_CHANNEL) +
            PRU_SUART_CH_TXRXSTATUS_OFFSET;

        pruss_readw(dev, offset, &regval);
        regval &= ~(CHN_TXRX_IE_MASK_PE);
        pruss_writew(dev, offset, regval);
    }

    return 0;
}

static s32 suart_set_pru_id(struct device *dev, u32 pru_no)
{
    u32 offset;
    u8 reg_val = 0;

    if (PRUSS_NUM0 == pru_no)
        offset = PRU_SUART_PRU0_ID_ADDR;
    else if (PRUSS_NUM1 == pru_no)
        offset = PRU_SUART_PRU1_ID_ADDR;
    else
        return -EINVAL;

    reg_val = pru_no;
    pruss_writeb(dev, offset, reg_val);
    return 0;
}
/* End of file */
