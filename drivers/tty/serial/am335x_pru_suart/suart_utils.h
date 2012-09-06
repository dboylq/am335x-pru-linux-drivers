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

#ifndef _SUART_UTILS_H_
#define _SUART_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 *====================
 * Includes
 *====================
 */
#include "tistdtypes.h"



/* Total number of baud rates supported */
#define SUART_NUM_OF_BAUDS_SUPPORTED	13


	extern void suart_mcasp_reset (arm_pru_iomap * pru_arm_iomap, unsigned int mcaspNum);
	extern void suart_mcasp_config(unsigned int mcasp_addr,
				       unsigned int txBaudValue,
				       unsigned int rxBaudValue,
				       unsigned int oversampling,
				       arm_pru_iomap * pru_arm_iomap,
                       unsigned int mcasp_num );

	extern short suart_asp_baud_set(unsigned int txBaudValue,
					unsigned int rxBaudValue,
					unsigned int oversampling,
					arm_pru_iomap * pru_arm_iomap, unsigned int mcaspNum);

	extern short suart_asp_serializer_deactivate (unsigned short u16srNum,
			 arm_pru_iomap * pru_arm_iomap, unsigned int mcaspNum);

	extern void suart_mcasp_tx_serialzier_set(unsigned int serializerNum,
			      arm_pru_iomap * pru_arm_iomap, unsigned int mcaspNum);
#ifdef __cplusplus
}				/* End of extern C */
#endif				/* #ifdef __cplusplus */
#endif
