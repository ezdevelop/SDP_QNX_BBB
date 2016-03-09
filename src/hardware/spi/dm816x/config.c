/*
 * $QNXLicenseC:
 * Copyright 2009, QNX Software Systems.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You
 * may not reproduce, modify or distribute this software except in
 * compliance with the License. You may obtain a copy of the License
 * at: http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" basis,
 * WITHOUT WARRANTIES OF ANY KIND, either express or implied.
 *
 * This file may contain contributions from others, either as
 * contributors under the License or as licensors under other terms.
 * Please review this entire file for other proprietary rights or license
 * notices, as well as the QNX Development Suite License Guide at
 * http://licensing.qnx.com/license-guide/ for other information.
 * $
 */

#include "omap3spi.h"

unsigned int speed_divisor[13] = {1,2,4,8,16,32,64,128,256,512,1024,2048,4096};

int omap3_cfg(void *hdl, spi_cfg_t *cfg)
{
	omap3_spi_t	*dev = hdl;
	uint32_t	ctrl;
	uint32_t	calculated_speed, i = 0;
   
	if (cfg == NULL)
		return 0;
	
	i = (cfg->mode & SPI_MODE_CHAR_LEN_MASK) - 1;
	
	if (i > 32 || i < 4)
		return 0;

	ctrl = (i << 7);
		
	if (!(cfg->mode & SPI_MODE_CSPOL_HIGH))
		ctrl |= OMAP3_MCSPI_CSPOL_ACTIVE_LOW;	/* Bit Order */ 
	
        if (!(cfg->mode & SPI_MODE_CKPOL_HIGH))
		ctrl |= OMAP3_MCSPI_POLARIY;	/* Active low polarity */
		
	if (cfg->mode & SPI_MODE_CKPHASE_HALF)
		ctrl |= OMAP3_MCSPI_PHASE;	/* CPHA 1 Phase */	

	/* Since all the modules here will be configured as SPI masters,
	 * the "somi" line is to be configured as "input/reception" line
	 * and "simo" has to be configured as "output/transmission" line
	 * we determine which line used as somi and which line as simo here.
	*/
	if (dev->somi) {		// SOMI: D1; SIMO; D0:
		ctrl |= OMAP3_MCSPI_IS_SOMI_INPUT_D1;
		ctrl &= OMAP3_MCSPI_DPE0_SIMO_OUTPUT;
		ctrl |= OMAP3_MCSPI_DPE1_TX_DISABLE;
	} else {				// SOMI: D0; SIMO; D1:
		ctrl &= OMAP3_MCSPI_IS_SOMI_INPUT;
		ctrl &= OMAP3_MCSPI_DPE1_SIMO_OUTPUT;
		ctrl |= OMAP3_MCSPI_DPE0_TX_DISABLE;
	}

        // Set the CS to start of data delay, default is 0.5 clcok cycle 
        ctrl &= OMAP3_MCSPI_CS_DELAY_MASK;
        ctrl |= (dev->cs_delay << 25 ); // 0, 1, 2, 3  bit 26:25
	
	/* Calculate the SPI target operational speed.
	 * The SPI module is supplie with a 48MHz reference clock.
	 * The SPI transfer speed has to be set by dividing this
	 * reference clock appropriately.
	 */
	 for(i = 0; i < 13;)
	 {
	 	calculated_speed = (dev->clock) / speed_divisor[i];
	 	if(calculated_speed <= cfg->clock_rate){
			//update cfg->clock_rate value as SPI_SCLK
		 	cfg->clock_rate = calculated_speed; 
	 		break;
	 	}
	 	i++;		
	 }		
	ctrl |= (i << 2);

 	return ctrl;
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.6.0/trunk/hardware/spi/dm816x/config.c $ $Rev: 697414 $")
#endif
