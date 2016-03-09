/*
 * $QNXLicenseC:  
 * Copyright 2009, QNX Software Systems. All Rights Reserved.
 *
 * This source code may contain confidential information of QNX Software 
 * Systems (QSS) and its licensors.  Any use, reproduction, modification, 
 * disclosure, distribution or transfer of this software, or any software 
 * that includes or is based upon any of this code, is prohibited unless 
 * expressly authorized by QSS by written agreement.  For more information 
 * (including whether this source code file has been published) please
 * email licensing@qnx.com. $
*/

/*
 *  hw/usbdc_vid_did.h
 *
 */

#ifndef __USBDC_VID_DID_H_INCLUDED
#define __USBDC_VID_DID_H_INCLUDED

#define USBDC_QNX_SOFTWARE_VENDOR_ID		0x1234

	#define USBDC_QNX_SOFTWARE_PRODUCT_ID_MSC		0x0001
	#define USBDC_QNX_SOFTWARE_PRODUCT_ID_CDC_ECM	0x0002
	#define USBDC_QNX_SOFTWARE_PRODUCT_ID_CDC_EEM	0x0003
	#define USBDC_QNX_SOFTWARE_PRODUCT_ID_CDC_ACM	0x0004
	#define USBDC_QNX_SOFTWARE_PRODUCT_ID_RNDIS		0x0005
	#define USBDC_QNX_SOFTWARE_PRODUCT_ID_HID		0x0006
	#define USBDC_QNX_SOFTWARE_PRODUCT_ID_AUDIO		0x0007
	#define USBDC_QNX_SOFTWARE_PRODUCT_ID_PRINTER	0x0008
	#define USBDC_QNX_SOFTWARE_PRODUCT_ID_CDC_OBEX	0x0009
	#define USBDC_QNX_SOFTWARE_PRODUCT_ID_TEST		0xFFF1

#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.6.0/trunk/hardware/devu/dc/public/hw/usbdc_vid_did.h $ $Rev: 680332 $")
#endif
