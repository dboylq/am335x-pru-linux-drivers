/*
 * pru/hal/uart/src/suart_utils.c
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
#include <linux/kernel.h>
#include "suart_pru_regs.h"
#include "am335x_suart_board.h"
#include "suart_api.h"
#include "suart_utils.h"
#include "suart_err.h"
#include "pru.h"
#include "suart_mcasp.h"


#define SUART_TRX_DIV_CONF_SZ	4

static short suart_mcasp_tx_baud_set(unsigned int txBaudValue,
			      arm_pru_iomap * pru_arm_iomap, unsigned int mcaspNum);
static short suart_mcasp_rx_baud_set(unsigned int rxBaudValue,
			      unsigned int oversampling,
			      arm_pru_iomap * pru_arm_iomap, unsigned int mcaspNum);

/*
 * Lookup table for TX baud rate
 * The divisor value is calculated using the formula
 *
 * ACLKX = (AUXCLK)/(CLKXDIV * HCLKXDIV)
 *
 * Where
 *		CLKXDIV takes values from 1-32
 *		HCLKXDIV takes values from 1-4096
 * Here
 *		AUXCLK = 24MHz
 */
static u32 lt_tx_baud_rate[][SUART_TRX_DIV_CONF_SZ] = {
    /*BaudRate,	Divisor,	CLKXDIV,HCLKXDIV */
    {300,		80000,		24,		3200},
    {600,		40000,		15,		2500},
    {1800,		13333,		10,		1212},
    {2400,		10000,		4,		2000},
    {4800,		5000,		1,		2500},
    {7200,		3333,		0,		3333},
    {9600,		2500,		0,		2500},
    {14400,		1666,		0,		1666},
    {19200,		1250,		0,		1250},
    {38400,		625,		0,		625},
    {57600,		416,		0,		416},
    {115200,	208,		0,		208},
    {230400,	104,		0,		104}
};

/*
 * Lookup table for RX baud rate for 8 bit oversampling
 * The divisor value is calculated using the formula
 *
 *	ACLKR = (AUXCLK)/(CLKRDIV * HCLKRDIV) * Oversampling
 *
 * Where
 *		CLKRDIV takes values from 1-32
 *		HCLKRDIV takes values from 1-4096
 * Here
 *		AUXCLK = 24MHz
 */
static u32 lt_rx_8x_baud_rate[][SUART_TRX_DIV_CONF_SZ] = {
    /* BaudRate,	Divisor,	CLKXDIV,	HCLKXDIV */
    {300,		10000,		4,		2000},
    {600,		5000,		1,		2500},
    {1800,		1667,		0,		1667},
    {2400,		1250,		0,		1250},
    {7200,		417,		0,		417},
    {4800,		625,		0,		625},
    {9600,		312,		0,		312},
    {14400,		208,		0,		208},
    {19200,		156,		0,		156},
    {38400,		78,	        0,      78},
    {57600,		52,	        0,      52},
    {115200,	26,	        0,      26},
    {230400,	13,	        0,      13}
};

/*
 * Lookup table for RX baud rate for 16 bit oversampling
 * The divisor value is calculated using the formula
 *
 *	ACLKR = (AUXCLK)/(CLKRDIV * HCLKRDIV) * Oversampling
 *
 * Where
 *		CLKRDIV takes values from 1-32
 *		HCLKRDIV takes values from 1-4096
 * Here
 *		AUXCLK = 24MHz
 */
static u32 lt_rx_16x_baud_rate[][SUART_TRX_DIV_CONF_SZ] = {
/*BaudRate,	Divisor,	CLKXDIV,	HCLKXDIV */
	{300,		5000,		1,		2500},
	{600,		2500,		0,		2500},
	{1800,		833,		0,		833},
	{2400,		625,		0,		625},
	{4800,		312,		0,		312},
	{7200,		208,		0,		208},
	{9600,		156,		0,		156},
	{14400,		104,		0,		104},
	{19200,		78,		    0,		78},
	{38400,		39,		    0,		39},
	{57600,		26,		    0,		26},
	{115200,	13,		    0,		13},
	{230400,	6,		    0,		6}
};

/*
 *====================
 * API implementations
 *====================
 */
/*
 * McASP configuration routine
 */

void suart_mcasp_reset (arm_pru_iomap * pru_arm_iomap, unsigned int mcaspNum)
{
    am335x_mcaspregsovly mcaspRegs;

    if(mcaspNum == 0)
        mcaspRegs = (am335x_mcaspregsovly) pru_arm_iomap->mcasp_io_addr;  //AM335X_MCASP_0_CTRL_REGS;
    else if(mcaspNum == 1)
        mcaspRegs = (am335x_mcaspregsovly) pru_arm_iomap->mcasp1_io_addr;  //AM335X_MCASP_0_CTRL_REGS;
    else
        return;

    // reset mcasp.
    mcaspRegs->gblctl = 0;
    mcaspRegs->rgblctl = 0;
    mcaspRegs->xgblctl = 0;
    mcaspRegs->xstat = 0x0000FFFF; // Clear all
    mcaspRegs->rstat = 0x0000FFFF; // Clear all

    return;
}

void suart_mcasp_config(unsigned int mcasp_addr,
			unsigned int txBaudValue,
			unsigned int rxBaudValue,
			unsigned int oversampling,
			arm_pru_iomap * pru_arm_iomap,
            unsigned int mcaspNum)
{
	am335x_mcaspregsovly mcaspRegs = (am335x_mcaspregsovly) mcasp_addr;	//AM335X_MCASP_0_CTRL_REGS;
    u32 i = 0, tmpVal = 0;

	// reset mcasp.
	mcaspRegs->gblctl = 0;
	mcaspRegs->rgblctl = 0;
	mcaspRegs->xgblctl = 0;

	// configure receive registers.
	if ((SUART_8X_OVRSMPL == oversampling) || (0 == oversampling)){
		mcaspRegs->rmask = 0x000000FF;
		mcaspRegs->rfmt = 0x0000A038;	//slot size 8 bits  , RPAD = 1.
	}
	if(SUART_16X_OVRSMPL == oversampling){
		mcaspRegs->rmask = 0x0000FFFF;
		mcaspRegs->rfmt = 0x0000A078;
	}

	mcaspRegs->afsrctl = 0x00000002;	//burst mode
	mcaspRegs->aclkrctl = 0x000000A0;
	mcaspRegs->ahclkrctl = 0x00008000;
	suart_mcasp_rx_baud_set(rxBaudValue, oversampling, pru_arm_iomap, mcaspNum);

	mcaspRegs->rtdm = 0x00000001;
	mcaspRegs->rintctl = 0x00000020;
	mcaspRegs->rclkchk = 0x00FF0008;

	// configure transmit registers.
	mcaspRegs->xmask = 0x0000FFFF;
	mcaspRegs->xfmt = 0x00002078;   //slot size 16 bits  , XPAD = 1.
	mcaspRegs->afsxctl = 0x0000002;  // Burst mode
	mcaspRegs->aclkxctl = 0x000000E0;
	mcaspRegs->ahclkxctl = 0x00008000;

	suart_mcasp_tx_baud_set(txBaudValue, pru_arm_iomap, mcaspNum);

	mcaspRegs->xtdm = 0x00000001;
	mcaspRegs->xintctl = 0x00000020;
	mcaspRegs->xclkchk = 0x00FF0008;

	//Serializer as a transmitter
	mcaspRegs->srctl0 = 0x000c;
	mcaspRegs->srctl1 = 0x000c;
	mcaspRegs->srctl2 = 0x000c;
	mcaspRegs->srctl3 = 0x000c;
#ifdef AM1808
	mcaspRegs->srctl4 = 0x000c;
	mcaspRegs->srctl5 = 0x000c;
	mcaspRegs->srctl6 = 0x000c;
	mcaspRegs->srctl7 = 0x000c;
	mcaspRegs->srctl8 = 0x000c;
	mcaspRegs->srctl9 = 0x000c;
	mcaspRegs->srctl10 = 0x000c;
	mcaspRegs->srctl11 = 0x000c;
	mcaspRegs->srctl12 = 0x000c;
	mcaspRegs->srctl13 = 0x000c;
	mcaspRegs->srctl14 = 0x000c;
	mcaspRegs->srctl15 = 0x000c;
#endif

	//Configure all AXR[n] as McASP pins

	/*
	 *  Setting  all TX MCASP AXR[n] Pin mapped to Even Serializer number (0,2,4,6,8,10,12,14)
	 *  to GPIO Mode by default. During setting the serializer to TX mode in PRU assembly code, the
	 *  MCASP AXR[n] Pin would get configured to MCASP mode of operation, before Actual Data Transfer.
	 */

    //Setting  all TX Pin to GPIO Mode by default
    for(i = 0; i < NR_SUART; i++) {
        if(mcaspNum == gSuartInfo[i].mcaspNum)
            tmpVal |= (1 << gSuartInfo[i].txSerializer);
    }

    mcaspRegs->pfunc = (AM335X_MCASP_PFUNC_RESETVAL) | tmpVal;

	mcaspRegs->pdout = 0xFFFF;

	// config pin function and direction.
	mcaspRegs->pdir = 0x00000000;
    mcaspRegs->pdir = (MCASP_PDIR_VAL) | tmpVal;

	mcaspRegs->pdout = 0xFFFF;

	mcaspRegs->ditctl = 0x00000000;
	mcaspRegs->dlbctl = 0x00000000;
	mcaspRegs->amute = 0x00000000;

	mcaspRegs->xstat = 0x0000FFFF;	// Clear all
	mcaspRegs->rstat = 0x0000FFFF;	// Clear all
}

void suart_mcasp_tx_serialzier_set(unsigned int serializerNum,
			      arm_pru_iomap * pru_arm_iomap, unsigned int mcaspNum)
{
	am335x_mcaspregsovly mcaspRegs;

        if(mcaspNum == 0)
            mcaspRegs = (am335x_mcaspregsovly) pru_arm_iomap->mcasp_io_addr;  //AM335X_MCASP_0_CTRL_REGS;
        else if(mcaspNum == 1)
            mcaspRegs = (am335x_mcaspregsovly) pru_arm_iomap->mcasp1_io_addr;  //AM335X_MCASP_0_CTRL_REGS;
        else
            return;

	mcaspRegs->pfunc |= (0x1 << serializerNum);
}

/*
 * mcasp TX buard rate setting routine
 */
short suart_mcasp_tx_baud_set(unsigned int txBaudValue,
			      arm_pru_iomap * pru_arm_iomap, unsigned int mcaspNum)
{
	unsigned int clkDivVal;
	unsigned int loopCnt;
	short status = SUART_SUCCESS;
	short foundVal = SUART_FALSE;
	am335x_mcaspregsovly mcaspRegs;


    if(mcaspNum == 0)
        mcaspRegs = (am335x_mcaspregsovly) pru_arm_iomap->mcasp_io_addr;  //AM335X_MCASP_0_CTRL_REGS;
    else if(mcaspNum == 1)
        mcaspRegs = (am335x_mcaspregsovly) pru_arm_iomap->mcasp1_io_addr;  //AM335X_MCASP_0_CTRL_REGS;
    else
        return -1;

	/* Search the supported baud rate in the table */
	for (loopCnt = 0; loopCnt < SUART_NUM_OF_BAUDS_SUPPORTED; loopCnt++) {
		if (txBaudValue == lt_tx_baud_rate[loopCnt][0]) {
			foundVal = SUART_TRUE;
			break;
		}
	}
	if (foundVal == SUART_TRUE) {
		clkDivVal = lt_tx_baud_rate[loopCnt][2];

		mcaspRegs->aclkxctl |= clkDivVal << AM335X_MCASP_ACLKXCTL_CLKXDIV_SHIFT;
		clkDivVal = lt_tx_baud_rate[loopCnt][3];	/* starts from 0 */
		mcaspRegs->ahclkxctl |= clkDivVal << AM335X_MCASP_AHCLKXCTL_HCLKXDIV_SHIFT;
	} else {
		return SUART_INVALID_TX_BAUD;
	}
	return (status);
}

/*
 * mcasp RX buard rate setting routine
 */
short suart_mcasp_rx_baud_set(unsigned int rxBaudValue,
			      unsigned int oversampling,
			      arm_pru_iomap * pru_arm_iomap, unsigned int mcaspNum)
{
    unsigned int clkDivVal;
    unsigned int loopCnt;
    short status = SUART_SUCCESS;
    short foundVal = SUART_FALSE;
    am335x_mcaspregsovly mcaspRegs;

    if(mcaspNum == 0)
        mcaspRegs = (am335x_mcaspregsovly) pru_arm_iomap->mcasp_io_addr;  //AM335X_MCASP_0_CTRL_REGS;
    else if(mcaspNum == 1)
        mcaspRegs = (am335x_mcaspregsovly) pru_arm_iomap->mcasp1_io_addr;  //AM335X_MCASP_0_CTRL_REGS;
    else
        return -1;

    if (oversampling == SUART_8X_OVRSMPL) {
        for (loopCnt = 0; loopCnt < SUART_NUM_OF_BAUDS_SUPPORTED;
                loopCnt++) {
            if (rxBaudValue == lt_rx_8x_baud_rate[loopCnt][0]) {
                clkDivVal = lt_rx_8x_baud_rate[loopCnt][2];
                mcaspRegs->aclkrctl |= clkDivVal << AM335X_MCASP_ACLKXCTL_CLKXDIV_SHIFT;
                clkDivVal = lt_rx_8x_baud_rate[loopCnt][3] - 1;	/* starts from 0 */
                mcaspRegs->ahclkrctl |= clkDivVal << AM335X_MCASP_AHCLKXCTL_HCLKXDIV_SHIFT;
                foundVal = SUART_TRUE;
                break;
            }
        }
    } else if (oversampling == SUART_16X_OVRSMPL) {
        for (loopCnt = 0; loopCnt < SUART_NUM_OF_BAUDS_SUPPORTED;
                loopCnt++) {
            if (rxBaudValue == lt_rx_16x_baud_rate[loopCnt][0]) {
                clkDivVal = lt_rx_16x_baud_rate[loopCnt][2];
                mcaspRegs->aclkrctl |= clkDivVal << AM335X_MCASP_ACLKXCTL_CLKXDIV_SHIFT;
                clkDivVal = lt_rx_16x_baud_rate[loopCnt][3] - 1; /* starts from 0 */
                mcaspRegs->ahclkrctl |= clkDivVal << AM335X_MCASP_AHCLKXCTL_HCLKXDIV_SHIFT;
                foundVal = SUART_TRUE;
                break;
            }
        }
    } else if (oversampling == 0) {
        for (loopCnt = 0; loopCnt < SUART_NUM_OF_BAUDS_SUPPORTED;
                loopCnt++) {
            if (rxBaudValue == lt_tx_baud_rate[loopCnt][0]) {
                clkDivVal = lt_tx_baud_rate[loopCnt][2];
                mcaspRegs->aclkrctl |= clkDivVal << AM335X_MCASP_ACLKXCTL_CLKXDIV_SHIFT;
                clkDivVal = lt_tx_baud_rate[loopCnt][3];
                mcaspRegs->ahclkrctl |= clkDivVal << AM335X_MCASP_AHCLKXCTL_HCLKXDIV_SHIFT;
                foundVal = SUART_TRUE;
                break;
            }
        }
    } else {
        return SUART_INVALID_OVERSAMPLING;
    }

    if (foundVal != SUART_TRUE) {
        return SUART_INVALID_RX_BAUD;
    }

    return (status);
}

/*
 * mcasp buard rate setting routine
 */
short suart_asp_baud_set(unsigned int txBaudValue,
			 unsigned int rxBaudValue,
			 unsigned int oversampling,
			 arm_pru_iomap * pru_arm_iomap, unsigned int mcaspNum)
{
	short status = SUART_SUCCESS;

	status = suart_mcasp_tx_baud_set(txBaudValue, pru_arm_iomap, mcaspNum);
	status = suart_mcasp_rx_baud_set(rxBaudValue, oversampling, pru_arm_iomap, mcaspNum);

	return (status);

}

/*
 * mcasp deactivate the selected serializer
 */
short suart_asp_serializer_deactivate (unsigned short u16srNum,
			 arm_pru_iomap * pru_arm_iomap, unsigned int mcaspNum)
{
	short status = SUART_SUCCESS;
	unsigned int * pu32SrCtlAddr = NULL;
    am335x_mcaspregsovly mcaspRegs;

    if(mcaspNum == 0)
        mcaspRegs = (am335x_mcaspregsovly) pru_arm_iomap->mcasp_io_addr;  //AM335X_MCASP_0_CTRL_REGS;
    else if(mcaspNum == 1)
        mcaspRegs = (am335x_mcaspregsovly) pru_arm_iomap->mcasp1_io_addr;  //AM335X_MCASP_0_CTRL_REGS;
    else
        return -1;

	if (u16srNum > 15)
	{
		status = SUART_INVALID_SR_NUM ;
	}
	else
	{
		pu32SrCtlAddr = (unsigned int *)&(mcaspRegs->srctl0);
		pu32SrCtlAddr += u16srNum;
		printk("\nline=%d\n",__LINE__);
		* (pu32SrCtlAddr) = 0x000C;
	}

	return (status);

}

/* End of file */
