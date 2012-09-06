/*
 * linux/<file location within the kernel tree>
 *
 * Copyright (C) 2010 Texas Instruments Incorporated
 * Author: Ganeshan N
 *
 * Based on <Give reference of old kernel file from which this file is derived from>
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

#ifndef _AM335X_SUART_BOARD_H_
#define _AM335X_SUART_BOARD_H_

#include <linux/mfd/pruss_core.h>

#ifdef __cplusplus
extern "C" {
#endif


/* ************ Serializers ***************** */
#define PRU_SUART_SERIALIZER_0          (0u)
/** Serializer */
#define PRU_SUART_SERIALIZER_1          (1u)
/** Serializer */
#define PRU_SUART_SERIALIZER_2          (2u)
/** Serializer */
#define PRU_SUART_SERIALIZER_3          (3u)
/** Serializer */
#define PRU_SUART_SERIALIZER_4          (4u)
/** Serializer */
#define PRU_SUART_SERIALIZER_5          (5u)
/** Serializer */
#define PRU_SUART_SERIALIZER_6          (6u)
/** Serializer */
#define PRU_SUART_SERIALIZER_7          (7u)
/** Serializer */
#define PRU_SUART_SERIALIZER_8          (8u)
/** Serializer */
#define PRU_SUART_SERIALIZER_9          (9u)
/** Serializer */
#define PRU_SUART_SERIALIZER_10         (10u)
/** Serializer */
#define PRU_SUART_SERIALIZER_11         (11u)
/** Serializer */
#define PRU_SUART_SERIALIZER_12         (12u)
/** Serializer */
#define PRU_SUART_SERIALIZER_13         (13u)
/** Serializer */
#define PRU_SUART_SERIALIZER_14         (14u)
/** Serializer */
#define PRU_SUART_SERIALIZER_15         (15u)
/** Serializer */
#define PRU_SUART_SERIALIZER_NONE       (16u)


#define MAX_SUARTS_SUPPORTED 4
typedef struct suart_info {
    u32 suartNum;
    u32 pruNum;
    u32 txPruChn;
    u32 rxPruChn;
    u32 txSysEvent;
    u32 rxSysEvent;
    u32 ctxBase;
    u32 fmtDataBase;
    u32 mcaspNum;
    u32 txSerializer;
    u32 rxSerializer;
}tSuartInfo;

/*
   Following are the configurable parameters in the AM335x SUART driver
   NR_SUART        -- Number of SUARTS, valid values are 1,2,3 and 4
   PRU0_MODE       -- Is PRU0 used(PRU_MODE_RX_TX_BOTH) or not used(PRU_MODE_INVALID)
   PRU1_MODE       -- Is PRU1 used(PRU_MODE_RX_TX_BOTH) or not used(PRU_MODE_INVALID)

   Description of few configurable parameters in the gSuartInfo table
   UART#           -- This has no significance in the table, added for readability
   PRU_NUM         -- PRU on which the UART has to RUN(Data from TX_CHN to DATA_BASE is depending on PRU_NUM)
   MCASP#          -- McASP on which the SUART is connected (Make sure it is enabled in menuconfig)
   TX_SER          -- Transmit serializer on the corresponding McASP
   RX_SER          -- Receiver serializer on the corresponding McASP

   To disable usage of PRU, change the PRUx_MODE and change TX_SER,
   RX_SER of the corresponding entries as PRU_SUART_SERIALIZER_NONE
 */

/* Number of SUARTS supported by the driver */
#define NR_SUART	4
#define PRU0_MODE    PRU_MODE_RX_TX_BOTH
#define PRU1_MODE    PRU_MODE_RX_TX_BOTH
static tSuartInfo gSuartInfo[MAX_SUARTS_SUPPORTED] =
{
    /* UART#    PRU_NUM  TX_CHN RX_CHN TX_EVT RX_EVT CTX_BASE DATA_BASE MCASP#                    TX_SER                    RX_SER*/
    {     1, PRUCORE_1,      0,     1,    24,    25,    0xB0,     0x90,     0,    PRU_SUART_SERIALIZER_2,    PRU_SUART_SERIALIZER_0},
    {     2, PRUCORE_1,      2,     3,    26,    27,   0x100,     0xE0,     0,    PRU_SUART_SERIALIZER_3,    PRU_SUART_SERIALIZER_1},

    {     3, PRUCORE_0,      0,     1,    20,    21,    0xB0,     0x90,     1,    PRU_SUART_SERIALIZER_1,    PRU_SUART_SERIALIZER_0},
    {     4, PRUCORE_0,      2,     3,    22,    23,   0x100,     0xE0,     1, PRU_SUART_SERIALIZER_NONE, PRU_SUART_SERIALIZER_NONE},
};

#ifdef __cplusplus
}				/* End of extern C */
#endif				/* #ifdef __cplusplus */
#endif				/* End of _<MOD>_FILENAME_API_H_ */
