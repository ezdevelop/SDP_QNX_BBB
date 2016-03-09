/*
 * $QNXLicenseC: 
 * Copyright 2010, QNX Software Systems.  
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


/* 
 *    Module - USB device controller driver for the Mentor MUSBMHDRC OTG USB controller
 *
 *	  Note 1:  Driver only support DMA for bulk endpoints, and only uses type 1
 *
 *    Note 2: This driver doesn't support double buffering... The DMA performance
 *            on bulk endpoints is very good with single buffering, so the additional
 *            complexity of double buffering is not required at this point.
 */


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>
#include <gulliver.h>
#include <sys/slog.h>
#include <hw/inout.h>
#include <drvr/common.h>
#include <descriptors.h>
#include <stdbool.h>

#include "musb.h"


extern char usbdc_config_descriptor[];


static iousb_pipe_methods_t musb_control_pipe_methods = {
	musb_control_endpoint_enable,
	musb_control_endpoint_disable,
	musb_control_transfer,
	musb_control_transfer_abort,
	NULL
};

static iousb_pipe_methods_t musb_interrupt_pipe_methods = {
	musb_endpoint_enable,
	musb_endpoint_disable,
	musb_transfer,
	musb_transfer_abort,
	NULL
};

static iousb_pipe_methods_t musb_bulk_pipe_methods = {
	musb_endpoint_enable,
	musb_endpoint_disable,
	musb_transfer,
	musb_transfer_abort,
	NULL
};

static iousb_pipe_methods_t musb_isoch_pipe_methods = {
	musb_isoch_endpoint_enable,
	musb_isoch_endpoint_disable,
	musb_isoch_transfer,
	musb_isoch_transfer_abort,
	NULL
};


dc_methods_t musb_controller_methods = {
		20,
		musb_init,
		musb_start,
		musb_stop,
		musb_shutdown,
		NULL,
		NULL,    
		musb_set_bus_state,
		musb_set_device_feature,
		musb_clear_device_feature,
		musb_set_device_address,
		musb_get_descriptor,
		musb_select_configuration,
		musb_interrupt,
		musb_set_endpoint_state,
		musb_clear_endpoint_state,
		NULL,
		&musb_control_pipe_methods,
		&musb_interrupt_pipe_methods,
		&musb_bulk_pipe_methods,
		&musb_isoch_pipe_methods,
};

usb_controller_methods musb_usb_controller_methods = {
	NULL,
	&musb_controller_methods,
	NULL,
	NULL
};

io_usb_dll_entry_t io_usb_dll_entry = {
	USBDC_DLL_NAME,
	0xffff,  // pci device id
	0xffff,
	0xffff,
	USB_CONTROLLER_DEVICE,
	NULL,
	NULL,
	&musb_usb_controller_methods
};

// local macros
#define	DIR_MASK 			0x80
#define HOST_TO_DEVICE		0x0

static void process_epout_interrupt( dctrl_t *dc, unsigned epidx );
static void process_ep0_interrupt( dctrl_t *dc );

// 53 USB spec defined test packet.
static uint8_t musb_testmode_pkt[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
									0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
									0xee, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
									0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
									0x7f, 0xbf, 0xdf, 0xef, 0xf7, 0xfb, 0xfd,
									0xfc, 0x7e, 0xbf, 0xdf, 0xef, 0xf7, 0xfb, 0xfd, 0x7e
								};

									
static unsigned musb_log2( unsigned n ) {
	unsigned cnt = 0;
	
	while( n != 1 ) {
		n = n >> 1;
		cnt++;
	}
	
	return cnt;
}


void 
musb_slogf( dctrl_t * dc, int level, const char *fmt, ...)
{
	va_list arglist;
	
	if ( dc && ( level > dc->verbosity ) ) 
		return; 
	
	va_start( arglist, fmt );
	vslogf( 12, level, fmt, arglist );
	va_end( arglist );
	return;
}

static void read_from_fifo(  dctrl_t * dc, unsigned fifonum, uint16_t count, uint8_t * rxbuf ) {
	
	// read as much as possible using 32-bit accesses
	while ( count >= 4 ) {
		*((uint32_t *)rxbuf) = HW_Read32( dc->IoBase, FIFO(fifonum) );
		count -= 4;
		rxbuf += 4;
	}
	
	// read the rest with 8-bit accesses
	while ( count ) {
		*rxbuf++ = HW_Read8( dc->IoBase, FIFO(fifonum) );
		count--;
	}
	
}


static void write_to_fifo(  dctrl_t * dc, unsigned fifonum, uint16_t count, uint8_t * txbuf ) {
	
	// read as much as possible using 32-bit accesses
	while ( count >= 4 ) {
		HW_Write32( dc->IoBase, FIFO(fifonum), *((uint32_t *)txbuf) );
		count -= 4;
		txbuf += 4;
	}
	
	// write the rest with 8-bit accesses
	while ( count ) {
		HW_Write8( dc->IoBase, FIFO(fifonum), *txbuf++ );
		count--;
	}
}

static void flush_fifo( dctrl_t * dc, ep_ctx_t *ep ) {
	uint16_t 	csr;
	uint32_t		csr_addr;
	uint16_t	flush_msk;
	uint16_t	rdy_msk;
	
	musb_slogf(dc, _SLOG_INFO, "%s() flushing epnum = %d epdir = 0x%x", __func__, ep->num, ep->dir  );
	
	if ( ep->num == 0 ) {
		csr = HW_Read16( dc->IoBase, MUSB_CSR0 ); 
		if ( csr & ( CSR0_RXPKTRDY | CSR0_TXPKTRDY ) ) {
			HW_Write16( dc->IoBase, MUSB_CSR0,CSR0_FLUSH_FIFO  );
		}
	} else {
		if ( ep->dir & USB_ENDPOINT_IN ) {
			csr_addr 	= MUSB_TXCSR( ep->num );
			flush_msk 	= TXCSR_FLUSHFIFO;
			rdy_msk		= TXCSR_TXPKTRDY;
		} else {
			csr_addr 	= MUSB_RXCSR( ep->num );
			flush_msk	= RXCSR_FLUSHFIFO;
			rdy_msk		= RXCSR_RXPKTRDY;
		}

		// flush fifo only if there is data in it, otherwise there may 
		// be data corruption as outlined in the programming guide
		csr = HW_Read16( dc->IoBase, csr_addr );
		if ( csr & rdy_msk ) {
			HW_Write16( dc->IoBase, csr_addr, flush_msk );
		}

#ifdef MUSB_DOUBLE_BUFFER_ENABLED
		{
			uint32_t	timeout = 1000;

			// wait until flush bit goes low before flushing again
			while( timeout-- && ( HW_Read16( dc->IoBase, csr_addr ) & flush_msk ) ) {
				usleep( 1 );
			}
			
			if ( timeout == 0 ) {
				musb_slogf(dc, _SLOG_INFO, "%s() flush bit is still active", __func__);
			}
		
			// flush again
			csr = HW_Read16( dc->IoBase, csr_addr );
			if ( csr & rdy_msk ) {
				HW_Write16( dc->IoBase, csr_addr, flush_msk );
			}
		}

#endif
	}
}


// !!!WARNING!!! this function assumes that it is called with driver mutex
// already locked
void complete_urb( dctrl_t * dc, ep_ctx_t *ep, uint32_t urb_status ) {
	iousb_transfer_t *urb = ep->urb;

	if ( urb ) {
		// we have sent/received everyting ... complete the urb
		urb->actual_len = ep->bytes_xfered;
		ep->urb = 0;
		MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "URB COMPLETE epnum = 0x%x epdir = 0x%x urb->actual_len = %d ",ep->num, ep->dir , urb->actual_len  );

		if( pthread_mutex_unlock( &dc->mutex ) )
			musb_slogf(dc, _SLOG_ERROR, "%s: mutex unlock failed ", __func__ );

		urb->urb_complete_cbf( ep->iousb_ep, urb, urb_status, 0);

		if( pthread_mutex_lock( &dc->mutex ) )
			musb_slogf(dc, _SLOG_ERROR, "%s: mutex lock failed ", __func__ );

	} else {
		musb_slogf(dc, _SLOG_ERROR, "URB COMPLETE ERROR urb = 0 epnum = 0x%x epdir = 0x%x ",ep->num, ep->dir  );
	}
}	

static void 
ep0_transmit( dctrl_t *dc ) {
	ep_ctx_t 	*ep = &dc->ep0;
	uint16_t	dataend = 0;	
	uint16_t	last_packet_in_xfer;
	
	// only transfer 1 usb packet at a time
	ep->req_xfer_len = ep->xfer_length - ep->bytes_xfered;
	if ( ep->req_xfer_len > ep->mps ) {
		// too much data for 1 usb packet... so cap it to mps
		ep->req_xfer_len = ep->mps;
		last_packet_in_xfer = 0;
	} else {
		last_packet_in_xfer = 1;
	}
	

	write_to_fifo( dc, 0, ep->req_xfer_len,  (uint8_t *) (ep->xfer_buffer + ep->bytes_xfered) );
	
	// set dataend if this is the last packet in transfer
	dataend = ( last_packet_in_xfer ) ? CSR0_DATAEND : 0;
	
	if ( last_packet_in_xfer ) {
		/* last usb packet of transfer... so hit DATAEND and
		 * transition the ep_state to IDLE so that the generated interrupt
		 * doesn't trigger another dataphase transfer
		 *
		 * Also: complete the URB *before* actual transfer goes out because
		 * it is very difficult ( if not impossible ) to to determine the state
		 * of the hardware once the interrupt fires because no actual status bits
		 * are set to tell us what happend.  The generated interrupt could mean
		 * the data phase completed, or if the interrupt latency is high enough, 
		 * then a bunch of events could have happended and captured by a single interrupt
		 * firing.  Therefore, for simplicity sake, complete the urb now: if the transfer
		 * in incomplete, then the HOST will take corrective action anyway
		 */
		
		dc->ep0.ep0_state = EP0_STATE_IDLE;	
		dc->ep0.ep0_idle_substate = EP0_IDLE_SUBSTATE_STATUS_PHASE;
		
		// modify to bytes_xfered to reflect the total number of bytes about
		// to be transmitted because we are completing the urb before the actual
		// transfer goes out.
		ep->bytes_xfered += ep->req_xfer_len;
		complete_urb( dc, ep, 0 );
		
		dataend = CSR0_DATAEND;
	} 

	MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: ep->req_xfer_len = %d dataend = 0x%x ", __func__ , ep->req_xfer_len, dataend );

	HW_Write16( dc->IoBase, MUSB_CSR0, CSR0_TXPKTRDY | dataend );

}

static void 
ep0_receive( dctrl_t *dc ) {
	// nothing to do... the controller will accept the next receive packet
	// and transfer it to the urb buffer in the interrupt handler.
}


static void 
ep_transmit( dctrl_t *dc, ep_ctx_t 	*ep ) {
	uint32_t		len;
	uint16_t        txcsr;
	
	if ( USE_DMA( ep ) && ( ep->xfer_length > 0 ) ) {
		txcsr = HW_Read16( dc->IoBase, MUSB_TXCSR(ep->num ) );
		if ( ( txcsr & TXCSR_DMA_REQ_EN ) == 0 ) {
			/* *** IMPORTANT ***
			* Only set the DMA_REQ_EN bit only if it isn't set.  The only way
			* this condition should happen is if we are coming off a ZLP transfer
			* in which case we are guaranteed that the TXPKTRDY bit will be low
			* because the ZLP will be completed by the TX EP interrupt.  
			* Otherwise ( if we unconditionally OR-IN the bit) , there is a
			* chance the TXPKTRDY bit may be set in which case a
			* read-modify-write of the bit may cause a zlp to go out over the bus
			* in the case where the hardware zeros the TXPKTRDY bit after the read
			* and we will be re-writting it to 1.  This condittion happens because
			* we complete the transfer from dma interrupt handler as we set
			* the TXPKTRDY bit to send out any residual.  We could complete the
			* residual from the endpoint interrupt, but we are taking extra interrupts
			* in this case and seems to be of little value
			*/
			
			HW_Write16( dc->IoBase, MUSB_TXCSR(ep->num ), txcsr | TXCSR_DMA_REQ_EN );
		}
		dc->dma_funcs.transfer_start( dc->dma_hdl, ep->dma_chidx, ep->xfer_buffer_paddr, ep->xfer_length, DMA_MODE1 );
	} else {
		// PIO TRANSFER... write packets into the fifo 1 at a time, trigger transmit,
		// take an interrupt and repeat until complete
		
		// disable the dma to make PIO work
		HW_Write16And( dc->IoBase, MUSB_TXCSR( ep->num ), ~(TXCSR_DMA_REQ_EN) & 0xffff );
		
		// only transfer 1 usb packet at a time
		len = ep->xfer_length - ep->bytes_xfered;
		if ( len > ep->mps ) {
			len = ep->mps;
		} 
		ep->req_xfer_len = len;	
	
		MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: len = %d ", __func__ , len);
		
		write_to_fifo( dc, ep->num , len,  (uint8_t *) (ep->xfer_buffer + ep->bytes_xfered) );

		HW_Write16Or( dc->IoBase, MUSB_TXCSR( ep->num ), TXCSR_TXPKTRDY );
	}
	
}


static void 
ep_receive( dctrl_t *dc, ep_ctx_t 	*ep ) {

	/* We cannot force an endpoint to NAK with the controller, so it is
	* possible the enpoint has already received a packet *before* the ep_receive()
	* was called. Therefore, see if we have already receive a packet for processing
	* This is most likely on the first packet, because we cannot use the poor man's NAK
	* because of boundary condition that there is no 0th packet
	*/
	
	if ( ep->flags & EP_FLAG_DEFER_OUT_PKT_PROCESSING ) {
		MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: Start defered processing for epnum = %d",__func__, ep->num );		
		process_epout_interrupt( dc, ep->num );
		ep->flags &= ~EP_FLAG_DEFER_OUT_PKT_PROCESSING;
	} else {
		
		if ( ep->flags & EP_FLAG_DEFER_OUT_PKT_CLEAR ) {
			// clear the RXPKTRDY bit for the previously received pkt as a way 
			// to clear the poor man's NAK
			HW_Write16And( dc->IoBase, MUSB_RXCSR( ep->num ), ~RXCSR_RXPKTRDY );	
			ep->flags &= ~EP_FLAG_DEFER_OUT_PKT_CLEAR;
		}
			
		if ( USE_DMA( ep ) ) {
			dc->dma_funcs.transfer_start( dc->dma_hdl, ep->dma_chidx, ep->xfer_buffer_paddr, ep->xfer_length, DMA_MODE1 );
		} 
	}
}


static void process_ep0_interrupt( dctrl_t *dc ) {
	uint16_t	csr;
	uint16_t	count;
	uint8_t		setup_packet[SIZE_SETUP_REQUEST];	
	ep_ctx_t 	*ep = &dc->ep0;
	static bool 	fAddrPending = false;
	static uint8_t 	fAddr = 0x0;

	
	csr = HW_Read16( dc->IoBase, MUSB_CSR0 );
	
	if ( csr & CSR0_SENTSTALL ) {
		HW_Write16( dc->IoBase, MUSB_CSR0, 0 );
	}

	if ( csr & CSR0_SETUPEND ) {
		HW_Write16( dc->IoBase, MUSB_CSR0, CSR0_SERVICED_SETUP_END );

		musb_slogf(dc, _SLOG_ERROR, "%s: CSR0_SETUPEND set", __func__ );

#if 0	// deal with early termination
		ep->flags &= ~EP_FLAG_DEFER_SETUP_PKT_CLEAR;
		dc->ep0.ep0_state = EP0_STATE_IDLE;	
		dc->ep0.ep0_idle_substate = EP0_IDLE_SUBSTATE_SETUP_PHASE;
#endif	
        // reset the flag if the status stage for the SET_ADDRESS command didn't complete
        fAddrPending = false;
	}
	
	if( ep->ep0_state == EP0_STATE_IDLE && fAddrPending ) {
		ep->ep0_idle_substate = EP0_IDLE_SUBSTATE_SETUP_PHASE;

		HW_Write8( dc->IoBase, MUSB_FADDR, (uint8_t) fAddr );
		fAddrPending = false;
		return;
	}
	
	switch ( ep->ep0_state ) {
		case EP0_STATE_IDLE:
			MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: state = EP0_STATE_IDLE csr = 0x%x",__func__, csr );		
			
			if ( csr & CSR0_RXPKTRDY ) {

				if ( ep->ep0_idle_substate == EP0_IDLE_SUBSTATE_SETUP_PHASE ) {
					MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: idle_substate = EP0_IDLE_SUBSTATE_SETUP_PHASE",__func__);		

					count = HW_Read16( dc->IoBase, MUSB_COUNT0 );
					
					if ( count == SIZE_SETUP_REQUEST ) {
						uint8_t  bmRequestType;
			
						// read the setup packet from the FIFO
						read_from_fifo( dc, 0, count, setup_packet );

						MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: count = %d rxpkt = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x"
							, __func__ , count ,setup_packet[0], setup_packet[1], setup_packet[2], setup_packet[3]
							, setup_packet[4], setup_packet[5], setup_packet[6], setup_packet[7] );
						
						
						/* The device controller will automatically complete the status phase.
						 * This is OK when the control transfer has a dataphase because the status phase won't
						 * complete until the data phase is explicitly terminated by the Driver by hitting
						 * "dataend".  However, this is a real problem form Control transfer with no data phases.
						 * In the case of USB_SET_CONFIGURATION because we want the function drivers to have a chance
						 * to configure themselves before completing the status phase. This is also the case
						 * for CLEAR_ENDPOINT_FEATURE which clears an endpoint stall.  The Host may
						 * think the stall is cleared before the driver actually gets the chance to clear it.
						 * Therefore, we will use the clearing of CSR0_SERVICED_RXPKTRDY to back-pressure the HOST in the
						 * case where the control transfer has no data phase
						 *
						 * Also, generalize the use of CSR0_SERVICED_RXPKTRDY for all host-device control transfers.  
						 * The reason for this is we cannot currently explicitly NAK the data-out phase of a control transfer,
						 * so we can use this as a poor man's NAK.  The reason this is usefull is in the case we can't to
						 * stall the data-out dataphase for a protocol stall on EP0
						 */
                        
						bmRequestType = setup_packet[0];
						if ( ( bmRequestType & DIR_MASK) == HOST_TO_DEVICE ) {
							uint8_t  bRequest;

							// control transfer with no data phase, or out-data phase
							// defer until status phase completed by transfer function
							MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: defer setup pkt clear",__func__);		
                            
							bRequest = setup_packet[1];
							if( bRequest == USB_SET_ADDRESS ) {
								fAddr = setup_packet[2];
								fAddrPending = true;
								HW_Write16( dc->IoBase, MUSB_CSR0, CSR0_SERVICED_RXPKTRDY | CSR0_DATAEND );
							} else {
								ep->flags |= EP_FLAG_DEFER_SETUP_PKT_CLEAR;
							}
						} else {
							// clear the rxpktrdy bit by setting the serviced bit as described in the ref manual
							HW_Write16( dc->IoBase, MUSB_CSR0, CSR0_SERVICED_RXPKTRDY );
						}

						/* transition the substate to the STATUS_PHASE because we will hit the status phase
						 * before the next SETUP phase regarless of whether the control transfer
						 * contains a data phase or not
						 */
						ep->ep0_idle_substate = EP0_IDLE_SUBSTATE_STATUS_PHASE;

						if( pthread_mutex_unlock( &dc->mutex ) )
							musb_slogf(dc, _SLOG_ERROR, "%s: mutex unlock failed ", __func__ );
						
						dc->udc->usbdc_self->usbdc_setup_packet_process( dc->udc, setup_packet );
						
						if( pthread_mutex_lock( &dc->mutex ) )
							musb_slogf(dc, _SLOG_ERROR, "%s: mutex lock failed ", __func__ );
						
					} else {
						
						if ( count > 0 ) {
							// Not sure what this data is becuase we are in the setup phase,
							// but the length is not 8 bytes so read it, but don't send the 
							// data anywhere, but log an error
	
							musb_slogf(dc, _SLOG_ERROR, "%s: Expected 8 bytes in the SETUP PHASE, but received %d... drop the data",__func__, count );	
	
							while ( count-- ) {
								HW_Read8( dc->IoBase, FIFO(0) );
							}
							HW_Write16( dc->IoBase, MUSB_CSR0, CSR0_SERVICED_RXPKTRDY );
						} 
/*						else {   
							
							It is possible and valid to get here via the following program flow:
							1. received a setup packet with an OUT data phase
							2. the setup packet rxpktrdy bit clear was delayed to give the stack the ability to stall, or nak until ready
							3. Stack stalls dataout phase
							!!!key!!!: the stall code doesn't attempt to clear the rxpktrdy bit, becase there is a race condition:
							The controller will accept the next setup packet before clearing the previous one, presumably because there is room in the fifo
							and the controller is not allowed to nak setup packets.  Clearing the rxpktrdy bit at this point could clobber the next
							setup packet
							4.  Receive a sentstall interrupt and clear it.
							5.  finally, we hit this else clause because we havven't cleared the rxpktrdy bit from the 
							last setup packet, and we havent't yet received the next setup packet.  We simply exit the handler and wait
							for the next ep0 interrupt
						}
*/


					}
				} else {
					MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: receive a packed in the IDLE state, but idle_substate != EP0_IDLE_SUBSTATE_SETUP_PHASE... defer processing",__func__);		
					ep->flags |= EP_FLAG_DEFER_SETUP_PKT_PROCESSING;
				}
			}

			break;	
		
		case EP0_STATE_TXDATA:
			MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: state = EP0_STATE_TXDATA csr = 0x%x",__func__, csr );		

			// still in the TXDATA state, so this means that we must have more data 
			// to send to the host
			ep->bytes_xfered += ep->req_xfer_len;
			ep0_transmit( dc );
			
			break;	
	
		case EP0_STATE_RXDATA:
			MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: state = EP0_STATE_RXDATA csr = 0x%x",__func__, csr );		
			if ( csr & CSR0_RXPKTRDY ) {
				if ( ep->urb == NULL ) {
					// should not hit this case because we are delaying the the clearing of the setup packet
					// until the dataout dataphase transfer function is called
					MUSB_SLOGF_DBG(dc, _SLOG_ERROR, "%s: NO URB", __func__ );
				} else {
					count =  HW_Read16( dc->IoBase, MUSB_COUNT0 );
					read_from_fifo( dc, 0, count, (uint8_t*) (ep->xfer_buffer + ep->bytes_xfered ) );
					ep->bytes_xfered += count;
							
					if ( ( ep->bytes_xfered >= ep->xfer_length) || ( count != ep->mps) ) {
						// either we recevied all the data we wanted, or we received a short packet,
						// ... either way, we are done receiving data  and punt it up the stack
						HW_Write16( dc->IoBase, MUSB_CSR0, CSR0_SERVICED_RXPKTRDY | CSR0_DATAEND );

						complete_urb( dc, ep, 0 );	
						ep->ep0_state = EP0_STATE_IDLE;
					} else {
						// more data to receive
						MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: More Data to receive  xfer_length = %d bytes_xfered = %d ", __func__, ep->xfer_length, ep->bytes_xfered  );
						HW_Write16( dc->IoBase, MUSB_CSR0, CSR0_SERVICED_RXPKTRDY );

						ep0_receive( dc);
					}
					
					
				}
			}
			break;	
		
	}
}

static void process_epin_interrupt( dctrl_t *dc, unsigned epidx ) {
	ep_ctx_t			*ep = &dc->epin_arr[epidx];
	uint16_t			txcsr;
	
	txcsr = HW_Read16( dc->IoBase, MUSB_TXCSR( epidx ) );
	MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: entry epidx = %d txcsr = 0x%x ", __func__ , epidx, txcsr );

	if ( txcsr & ( TXCSR_SENT_STALL | TXCSR_UNDERRUN ) ) {
		
		HW_Write16And( dc->IoBase, MUSB_TXCSR( ep->num ), ~(TXCSR_SENT_STALL | TXCSR_UNDERRUN ) );		
	} 
	
	
	if ( ep->urb ) {
		if ( txcsr & TXCSR_TXPKTRDY ) {
			musb_slogf(dc, _SLOG_ERROR, "%s: TXCSR_TXPKTRDY still set", __func__ );
		}
		
		ep->bytes_xfered += ep->req_xfer_len;
	
		if ( ep->bytes_xfered < ep->xfer_length) {
				MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: More Data to transmit  xfer_length = %d bytes_xfered = %d ", __func__, ep->xfer_length, ep->bytes_xfered  );
				ep_transmit( dc, ep );
		} else {
				complete_urb( dc, ep, 0 );	
		}
	} 
}

static void process_epout_interrupt( dctrl_t *dc, unsigned epidx ) {
	uint16_t			rxcsr;
	uint16_t			rxcount;
	ep_ctx_t			*ep = &dc->epout_arr[epidx];
	
	rxcsr = HW_Read16( dc->IoBase, MUSB_RXCSR( epidx ) );
	MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: entry epidx = %d  rxcsr = 0x%x", __func__ , epidx, rxcsr );

	if ( rxcsr & RXCSR_SENT_STALL ) {
		HW_Write16And( dc->IoBase, MUSB_RXCSR( ep->num ), ~RXCSR_SENT_STALL );
	}	
	
	if ( rxcsr & RXCSR_RXPKTRDY ) {
			
		if ( USE_DMA( ep ) && ( ( ep->flags & EP_FLAG_DEFER_OUT_PKT_PROCESSING ) == 0 ) ) {
			// do DMA post processing... 2 possible scenarios
			// Case1. DMA did not fully complete 
			// ( short packet received and didn't trigger an interrupt ),
			// so figure out how much data was DMA'ed  
			//
			// Case2:  DMA fully commpleted ( last packet was mps ), and urb
			// buffer is completely filled and needs to be completed, but the
			// a short packet was received ( belonging to the next transfer ) 
			// and generated this interrupt first... In this case, the rx_complete()
			// will complete the previous urb and set ep->urb to NULL.  Therefore,
			// the packet which generated this interrupt will be processed via
			// DEFERED PROCESSING.
			//
			// !!!WARNING!!!. We do no call rx_complete() when we are doing defered processing,
			// because the packet was received before the dma could have potentially started and thus 
			// was not started in the ep_recieve() transfer function
			ep->bytes_xfered = dc->dma_funcs.rx_complete( dc->dma_hdl, ep->dma_chidx );
		} 
		
		if ( ep->urb == NULL ) {
			MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: NO URB -defer processing until transfer function is called", __func__ );
			ep->flags |= EP_FLAG_DEFER_OUT_PKT_PROCESSING;
		} else {

			// how many bytes are in the fifo
			rxcount = HW_Read16( dc->IoBase, MUSB_RXCOUNT( epidx ) );
			
			read_from_fifo( dc, epidx, rxcount, (uint8_t*) (ep->xfer_buffer + ep->bytes_xfered ) );
			ep->bytes_xfered += rxcount;
			
			if ( ( ep->bytes_xfered >= ep->xfer_length) || ( rxcount != ep->mps) ) {
				// either we recevied all the data we wanted, or we received a short packet,
				// ... either way, we are done receiving data  and punt it up the stack
				ep->flags |= EP_FLAG_DEFER_OUT_PKT_CLEAR;
				complete_urb( dc, ep, 0 );	

				// !!! Don't clear the RXPKTRDY bit here!!!! clear it when we
				// receive the next out transfer function as a poor man's NAK

			} else {
				// more data to receive

				// clear the rxpktrdy bit *after* rx_cleanup to make sure a new
				// packet isn't received and processed by the dma engine accidentally.			
				HW_Write16And( dc->IoBase, MUSB_RXCSR( epidx ), ~RXCSR_RXPKTRDY );	

				MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: More Data to receive  xfer_length = %d bytes_xfered = %d ", __func__, ep->xfer_length, ep->bytes_xfered  );
			}
			
		}
	}
}




uint32_t musb_interrupt( usbdc_device_t *udc )
{
	dctrl_t  	*dc = udc->dc_data;
	uint8_t		usb_intstatus;
	uint16_t	tx_intstatus;
	uint16_t	rx_intstatus;
	int i;
	uint8_t		power;

	
	if( pthread_mutex_lock( &dc->mutex ) )
		musb_slogf(dc, _SLOG_ERROR, "%s: mutex lock failed ", __func__ );

#ifdef EXTERNAL_INTERRUPT_PROCESSING
	musb_get_ext_intstatus( dc, &rx_intstatus, &tx_intstatus, &usb_intstatus);
#else
	// read interrupt status registers
	usb_intstatus = HW_Read8( dc->IoBase, MUSB_INTRUSB );
	tx_intstatus = HW_Read16( dc->IoBase, MUSB_INTRTX );
	rx_intstatus = HW_Read16( dc->IoBase, MUSB_INTRRX );
#endif

	MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s:  usb_intstatus = 0x%x rx_intstatus = 0x%x tx_intstatus = 0x%x ",__func__, usb_intstatus, rx_intstatus, tx_intstatus);

	
	if ( usb_intstatus & INTRUSB_RESET ) {
		MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: RESET INTERRUPT ",__func__ );
		
		// check to see if transfer in progress
		// this is to determine user impact of a hardware issue when transferring 0xAAAA pattern to the host
		for( i=0; i < dc->musbmhdrc_cfg.n_endpoints; i++ ) {
			if( dc->epin_arr[i].urb != 0 ) {
				musb_slogf(dc, _SLOG_WARNING, "%s: Got RESET INTERRUPT while ep %d still in progress", __func__, i );
			}
		}
		
		// Use the USB Reset Interrupt as an indication that the device
		// was inserted. 
		if ( ( dc->flags & DC_FLAG_CONNECTED ) == 0 ) {
			dc->flags |= DC_FLAG_CONNECTED;
			
			udc->usbdc_self->usbdc_device_state_change( udc, IOUSB_DEVICE_STATE_INSERTED);
			musb_slogf(dc, _SLOG_INFO, "%s: IOUSB_DEVICE_STATE_INSERTED", __func__ );	
		}
		
		// check and report the speed
		power = HW_Read8( dc->IoBase, MUSB_POWER );
		if ( power & POWER_HS_MODE ) {
			musb_slogf(dc, _SLOG_INFO, "%s: HIGH SPEED DETECTED", __func__ );	
			udc->usbdc_self->usbdc_set_device_speed( udc, IOUSB_DEVICE_HIGH_SPEED ); 
			
		} else {
			musb_slogf(dc, _SLOG_INFO, "%s: FULL SPEED DETECTED", __func__ );
			udc->usbdc_self->usbdc_set_device_speed( udc, IOUSB_DEVICE_FULL_SPEED ); 
		}
		
		// report the reset *after* the speed has been reported
		udc->usbdc_self->usbdc_device_state_change( udc, IOUSB_DEVICE_STATE_RESET );

		// Capture the fact that EP0 is back in the IDLE state after a USB reset 
		dc->ep0.ep0_state = EP0_STATE_IDLE;
		dc->ep0.ep0_idle_substate = EP0_IDLE_SUBSTATE_SETUP_PHASE;
		
	}
	
	if ( usb_intstatus & INTRUSB_DISCON ) {
		MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: DISCONNECT INTERRUPT ",__func__ );
		
		udc->usbdc_self->usbdc_device_state_change( udc, IOUSB_DEVICE_STATE_REMOVED);
		dc->flags &= ~DC_FLAG_CONNECTED;
	} 
	
	if ( usb_intstatus & INTRUSB_SUSPEND ) {
		if ( dc->flags & DC_FLAG_CONNECTED ) {
			MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: SUSPEND INTERRUPT ",__func__ );
			if ( udc->usbdc_self->usbdc_device_state_change( udc, IOUSB_DEVICE_SUSPEND_REQUEST) == EOK ) { 
			}
			udc->usbdc_self->usbdc_device_state_change( udc, IOUSB_DEVICE_STATE_SUSPENDED );
		} else {
			MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: SPURIOUS SUSPEND INTERRUPT... we are not connected",__func__ );
		}
	}
	
	if ( usb_intstatus & INTRUSB_RESUME ) {
		MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: RESUME INTERRUPT ",__func__ );
		udc->usbdc_self->usbdc_device_state_change( udc, IOUSB_DEVICE_STATE_RESUMED );
	} 

	if ( tx_intstatus & EP_INT_MSK( 0 ) ) {
		// ep0 interrupt
		tx_intstatus &= ~EP_INT_MSK( 0 );  //clr
		process_ep0_interrupt( dc );
	}

	// process epin interrupts
	if ( tx_intstatus ) {
		for(i=1; i < dc->musbmhdrc_cfg.n_endpoints; i++ ) {
			if ( tx_intstatus & EP_INT_MSK( i ) ) {
				process_epin_interrupt( dc, i );
			}
		}
	}
	
	// process epout interrupts
	if ( rx_intstatus ) {
		for(i=1; i < dc->musbmhdrc_cfg.n_endpoints; i++ ) {
			if ( rx_intstatus & EP_INT_MSK( i ) ) {
				process_epout_interrupt( dc, i );
			}
		}
	}
	
#ifdef EXTERNAL_INTERRUPT_PROCESSING
	musb_clr_ext_int( dc );
#endif
	
	if( pthread_mutex_unlock( &dc->mutex ) )
		musb_slogf(dc, _SLOG_ERROR, "%s: mutex unlock failed ", __func__ );
	
	
	return EOK;
}


uint32_t
musb_set_device_address(  usbdc_device_t *udc, uint32_t address )
{
	dctrl_t  	*dc = udc->dc_data;

	musb_slogf(dc, _SLOG_INFO, "%s: address = 0x%x", __func__, address);
	
	/* the status phase is completed automatically by the device controller,
	 * but there is no transfer function issued by io-usb so transition
	 * the substate such that the next setup packet can be processed
	 */
	dc->ep0.ep0_idle_substate = EP0_IDLE_SUBSTATE_SETUP_PHASE;

	return EOK; 
}


uint32_t
musb_select_configuration( usbdc_device_t *udc, uint8_t config )
{
	dctrl_t  *dc = udc->dc_data;
		
	musb_slogf(dc, _SLOG_INFO, "%s: config = 0x%x", __func__, config);
	
	return EOK;
}


uint32_t
musb_set_endpoint_state( usbdc_device_t *udc, iousb_endpoint_t *iousb_ep, uint32_t ep_state )
{
	dctrl_t 		*dc 		= udc->dc_data;
	ep_ctx_t		*ep = iousb_ep->user;
	
	musb_slogf(dc, _SLOG_INFO, "%s: state = %d",__func__, ep_state );
	
	if( pthread_mutex_lock( &dc->mutex ) )
		musb_slogf(dc, _SLOG_ERROR, "%s: mutex lock failed ", __func__ );
	
	switch ( ep_state ) {
		case IOUSB_ENDPOINT_STATE_READY :
			musb_slogf(dc, _SLOG_INFO, "%s: IOUSB_ENDPOINT_STATE_READY epnum = 0x%x", __func__, iousb_ep->edesc.bEndpointAddress);
			break;
		
		case IOUSB_ENDPOINT_STATE_STALLED :
			musb_slogf(dc, _SLOG_INFO, "          STALL epnum = 0x%x",  iousb_ep->edesc.bEndpointAddress);
			if ( ep->num == 0 ) {
				HW_Write16( dc->IoBase, MUSB_CSR0, CSR0_SENDSTALL  );

				// expecting a setup packet next
				ep->flags &= ~EP_FLAG_DEFER_SETUP_PKT_CLEAR;
				dc->ep0.ep0_state = EP0_STATE_IDLE;	
				dc->ep0.ep0_idle_substate = EP0_IDLE_SUBSTATE_SETUP_PHASE;
			} else {
				if ( ep->dir == USB_ENDPOINT_IN ) {
					// this bit must be cleared by the CPU.. not autocleared
					HW_Write16Or( dc->IoBase, MUSB_TXCSR( ep->num ), TXCSR_SEND_STALL );
				} else {
					// this bit must be cleared by the CPU.. not autocleared
					HW_Write16Or( dc->IoBase, MUSB_RXCSR( ep->num ), RXCSR_SEND_STALL );
				}
			}
			
			break;
		
		case IOUSB_ENDPOINT_STATE_RESET :
			musb_slogf(dc, _SLOG_INFO, "%s: IOUSB_ENDPOINT_STATE_RESET epnum = 0x%x", __func__, iousb_ep->edesc.bEndpointAddress);
			break;
		case IOUSB_ENDPOINT_STATE_ENABLE :
			musb_slogf(dc, _SLOG_INFO, "%s: IOUSB_ENDPOINT_STATE_ENABLE epnum = 0x%x", __func__, iousb_ep->edesc.bEndpointAddress);
			break;

		case IOUSB_ENDPOINT_STATE_DISABLED :
			musb_slogf(dc, _SLOG_INFO, "%s: IOUSB_ENDPOINT_STATE_DISABLED epnum = 0x%x", __func__, iousb_ep->edesc.bEndpointAddress);
			break;
		
		case IOUSB_ENDPOINT_STATE_NAK :
			musb_slogf(dc, _SLOG_INFO, "%s: IOUSB_ENDPOINT_STATE_NAK epnum = 0x%x", __func__, iousb_ep->edesc.bEndpointAddress);
			break;
		

		default :
			break;
	}
	
	if( pthread_mutex_unlock( &dc->mutex ) )
	musb_slogf(dc, _SLOG_ERROR, "%s: mutex lock failed ", __func__ );
	
	return EOK;
}

uint32_t
musb_clear_endpoint_state( usbdc_device_t *udc, iousb_endpoint_t *iousb_ep, uint32_t ep_state )
{
	dctrl_t 		*dc = udc->dc_data;
	ep_ctx_t		*ep = iousb_ep->user;
	uint16_t		csr;
	
	if( pthread_mutex_lock( &dc->mutex ) )
		musb_slogf(dc, _SLOG_ERROR, "%s: mutex lock failed ", __func__ );
	
	switch ( ep_state ) {
		case IOUSB_ENDPOINT_STATE_READY :
			musb_slogf(dc, _SLOG_INFO, "%s: CLEAR IOUSB_ENDPOINT_STATE_READY  epnum = 0x%x", __func__, iousb_ep->edesc.bEndpointAddress);
			break;
		
		case IOUSB_ENDPOINT_STATE_STALLED :
			musb_slogf(dc, _SLOG_INFO, "          Clear STALL  epnum = 0x%x",  iousb_ep->edesc.bEndpointAddress);

			// ep0 stall are self clearing
			if ( ep != NULL && ep->num != 0 ) {
				if ( ep->dir == USB_ENDPOINT_IN ) {
					csr = HW_Read16( dc->IoBase, MUSB_TXCSR( ep->num ) ) & ~( TXCSR_SEND_STALL | TXCSR_SENT_STALL );
					csr |= TXCSR_CLR_DATA_TOGGLE;
					HW_Write16( dc->IoBase, MUSB_TXCSR( ep->num ), csr );
					MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "  tried to clear actual stall txcsr = 0x%x" , HW_Read16( dc->IoBase, MUSB_TXCSR( ep->num ) )  );
					
				} else {
					csr = HW_Read16( dc->IoBase, MUSB_RXCSR( ep->num ) ) & ~( RXCSR_SEND_STALL | RXCSR_SENT_STALL | RXCSR_RXPKTRDY );
					csr |= RXCSR_CLR_DATA_TOGGLE;
					HW_Write16( dc->IoBase, MUSB_RXCSR( ep->num ), csr );
					MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "  tried to clear actual stall rxcsr = 0x%x" , HW_Read16( dc->IoBase, MUSB_RXCSR( ep->num ) )  );
				}
			}
			break;
		
		case IOUSB_ENDPOINT_STATE_RESET :
			musb_slogf(dc, _SLOG_INFO, "%s: CLEAR IOUSB_ENDPOINT_STATE_RESET  epnum = 0x%x", __func__, iousb_ep->edesc.bEndpointAddress);
			break;

		case IOUSB_ENDPOINT_STATE_ENABLE :
			musb_slogf(dc, _SLOG_INFO, "%s: CLEAR IOUSB_ENDPOINT_STATE_ENABLE  epnum = 0x%x", __func__, iousb_ep->edesc.bEndpointAddress);
			break;

		case IOUSB_ENDPOINT_STATE_DISABLED :
			musb_slogf(dc, _SLOG_INFO, "%s: CLEAR IOUSB_ENDPOINT_STATE_DISABLED  epnum = 0x%x", __func__, iousb_ep->edesc.bEndpointAddress);
			break;

		case IOUSB_ENDPOINT_STATE_NAK :		
			musb_slogf(dc, _SLOG_INFO, "%s: CLEAR IOUSB_ENDPOINT_STATE_NAK  epnum = 0x%x", __func__, iousb_ep->edesc.bEndpointAddress);
			break;

		default :
			break;
	}        
	
	if( pthread_mutex_unlock( &dc->mutex ) )
		musb_slogf(dc, _SLOG_ERROR, "%s: mutex lock failed ", __func__ );
	
	return EOK;
}



/* call to enable connection to host or signal resume to host */
uint32_t
musb_set_bus_state( usbdc_device_t *udc, uint32_t device_state )
{
	dctrl_t  *dc = udc->dc_data;
	
	if( pthread_mutex_lock( &dc->mutex ) ) {
		musb_slogf(dc, _SLOG_ERROR, "%s: mutex lock failed ", __func__ );
	}
	
	switch ( device_state ) {
	case IOUSB_BUS_STATE_DISCONNECTED :
		HW_Write8And( dc->IoBase, MUSB_POWER, ~POWER_SOFT_CONN );
		udc->usbdc_self->usbdc_device_state_change( udc, IOUSB_DEVICE_STATE_REMOVED);
		dc->flags &= ~DC_FLAG_CONNECTED;
		
		break;
		
	case IOUSB_BUS_STATE_CONNECTED :
		HW_Write8Or( dc->IoBase, MUSB_POWER, POWER_SOFT_CONN );
		break;
		
	case IOUSB_BUS_STATE_RESUME :
		break;
	}
	
	if( pthread_mutex_unlock( &dc->mutex ) ) {
		musb_slogf(dc, _SLOG_ERROR, "%s: mutex unlock failed ", __func__ );
	}
	
	return EOK;
}

/*
	Set port test mode. This must be done witin 3ms of status phase of request.
*/

uint32_t
musb_set_test_mode( dctrl_t *dc, uint16_t wIndex ) 
{
	uint8_t		mode;

	musb_slogf(dc, 7, "%s : 0x%x", __func__, wIndex );

	switch( wIndex ) {	
		case USB_TEST_MODE_TEST_J :
			mode = MUSB_TEST_TEST_J;
			break;

		case USB_TEST_MODE_TEST_K :
			mode = MUSB_TEST_TEST_K;
			break;

		case USB_TEST_MODE_TEST_SE0_NAK :
			mode = MUSB_TEST_SE0_NAK;
			break;

		case USB_TEST_MODE_TEST_PACKET :
			mode = MUSB_TEST_PACKET;
			write_to_fifo( dc, 0, sizeof(musb_testmode_pkt), musb_testmode_pkt );
			HW_Write16( dc->IoBase, MUSB_CSR0, CSR0_TXPKTRDY );
			break;

		case USB_TEST_MODE_TEST_FORCE_ENABLE : // only vald for downstream port(Host)
		default :	
			return( ENOTSUP );	
	}

	HW_Write8( dc->IoBase, MUSB_TESTMODE, mode );

	return( EOK );
}

/* enable a feature of a device */
uint32_t
musb_set_device_feature( usbdc_device_t *udc, uint32_t feature, uint16_t wIndex )
{
	dctrl_t  *dc = udc->dc_data;

	switch ( feature ) {
		case USB_FEATURE_DEV_WAKEUP :
			return( ENOTSUP );
			break;

		case USB_FEATURE_TEST_MODE :
			return( musb_set_test_mode( dc, wIndex ) );
			break;

		default :
			return( ENOTSUP );
			break;
	}

	return( ENOTSUP );
}

/* clear a feature of a device */
uint32_t
musb_clear_device_feature( usbdc_device_t *udc, uint32_t feature )
{
	switch ( feature ) {
		case USB_FEATURE_DEV_WAKEUP :
			return( ENOTSUP );
			break;

		case USB_FEATURE_TEST_MODE : // don't support clearing of test modes(will never get here)
		default :
			return( ENOTSUP );
	}

	return( ENOTSUP );
}

uint32_t
musb_get_descriptor( usbdc_device_t *udc, uint8_t type, uint8_t index, uint16_t lang_id, uint8_t **desc, uint32_t speed )
{	
	dctrl_t *dc = udc->dc_data;

	switch ( type ) {
	case USB_DESC_DEVICE :
		*desc = (speed == IOUSB_DEVICE_HIGH_SPEED) ? (uint8_t *) USBDC_HS_DEVICE_DESCRIPTOR : (uint8_t *) USBDC_FS_DEVICE_DESCRIPTOR;
		musb_slogf(dc, _SLOG_INFO, "%s : get USBDC_DEVICE_DESCRIPTOR", __func__);
		break;
	
	case USB_DESC_CONFIGURATION :
		if ( index < USBDC_NUM_CONFIGURATIONS ) {
			musb_slogf(dc, _SLOG_INFO, "%s : get USBDC_CONFIG_DESCRIPTOR speed = %d index = %d", __func__,speed,index);
			*desc  = (speed == IOUSB_DEVICE_HIGH_SPEED) ? (uint8_t *) USBDC_HS_CONFIG_DESCRIPTOR[index] : (uint8_t *) USBDC_FS_CONFIG_DESCRIPTOR[index];
		} else {
			return( ENOTSUP );
		}
		break;
	
	case USB_DESC_STRING :
		if ( index <= USBDC_MAX_STRING_DESCRIPTOR ) {
			musb_slogf(dc, _SLOG_INFO, "%s : get USBDC_STRING_DESCRIPTOR idx=%d", __func__,index);
			*desc = (speed == IOUSB_DEVICE_HIGH_SPEED) ? (uint8_t *) USBDC_HS_STRING_DESCRIPTOR[index] : (uint8_t *) USBDC_FS_STRING_DESCRIPTOR[index];
		} else {
			return ENOTSUP;
		}
		break;
	
	case USB_DESC_DEVICE_QUALIFIER :
		musb_slogf(dc, _SLOG_INFO, "%s : get USB_DESC_DEVICE_QUALIFIER", __func__);
		*desc = (speed == IOUSB_DEVICE_HIGH_SPEED) ? (uint8_t *) USBDC_HS_DEVICE_QUALIFIER_DESCRIPTOR : (uint8_t *) USBDC_FS_DEVICE_QUALIFIER_DESCRIPTOR;
		break;
		
	case USB_DESC_OTHER_SPEED_CONF :
		musb_slogf(dc, _SLOG_INFO, "%s : get USB_DESC_OTHER_SPEED_CONF speed = %d index = %d", __func__, speed, index);
		*desc  = (speed == IOUSB_DEVICE_HIGH_SPEED) ? (uint8_t *) USBDC_HS_CONFIG_DESCRIPTOR[0] : (uint8_t *) USBDC_FS_CONFIG_DESCRIPTOR[0];
		break;
		
		
	case USB_DESC_INTERFACE_POWER :
	case USB_DESC_INTERFACE :
	case USB_DESC_ENDPOINT :	
	default :
		return ENOTSUP;
		break;
	}
	return EOK;
}

_uint32
musb_control_endpoint_enable( void *chdl, iousb_device_t *device, iousb_endpoint_t *iousb_ep )
{
	usbdc_device_t *udc = 	( usbdc_device_t * ) chdl;
	dctrl_t * dc = 			( dctrl_t * ) udc->dc_data;
	ep_ctx_t				*ep = iousb_ep->user;

	musb_slogf(dc, _SLOG_INFO, "%s: ep = 0x%x", __func__, iousb_ep->edesc.bEndpointAddress );

	iousb_ep->flags = IOUSB_ENDPOINT_FLAGS_BMAP;	// no dma 
	
	if ( !iousb_ep->user ) {
		//create endpoint context
		ep = iousb_ep->user = &dc->ep0;
		ep->iousb_ep = iousb_ep;	// backref
		ep->mps = iousb_ep->edesc.wMaxPacketSize;
		ep->num = iousb_ep->edesc.bEndpointAddress & 0x7f;
		ep->type = iousb_ep->edesc.bmAttributes & 3;
	
	}
	
	// reset some state variables
	ep->urb = 0;
	ep->xfer_buffer = 0;
	ep->xfer_length = 0;
	ep->xfer_flags = 0;
	ep->req_xfer_len = 0;
	
	return ( EOK );
	
}


// used by both bulk and interrupt endpoints
_uint32 
musb_endpoint_enable( void *chdl, iousb_device_t *device, iousb_endpoint_t *iousb_ep )
{
	usbdc_device_t 	*udc = ( usbdc_device_t * ) chdl;
	dctrl_t 		*dc = ( dctrl_t * ) udc->dc_data;
	ep_ctx_t		*ep = iousb_ep->user; 
	uint32_t		epnum = iousb_ep->edesc.bEndpointAddress & 0x7f;
	uint32_t		epdir = iousb_ep->edesc.bEndpointAddress & USB_ENDPOINT_IN;
	uint32_t		fifosz_in_bytes;
	uint8_t			double_buffer_flag = 0;
	uint8_t			v;
	
	musb_slogf(dc, _SLOG_INFO, "%s: ep = 0x%x mps = %d", __func__, iousb_ep->edesc.bEndpointAddress, iousb_ep->edesc.wMaxPacketSize);
	
	if ( epnum >= dc->musbmhdrc_cfg.n_endpoints ) {
		musb_slogf(dc, _SLOG_ERROR, "%s: epnum %d is too large.. must be < %d", __func__, epnum, dc->musbmhdrc_cfg.n_endpoints );
		return EINVAL;
	}
	
	if ( !iousb_ep->user ) {
		ep = iousb_ep->user = ( epdir == USB_ENDPOINT_IN ) ? &dc->epin_arr[epnum] : &dc->epout_arr[epnum];
		ep->iousb_ep = iousb_ep;	// backref
		ep->mps = iousb_ep->edesc.wMaxPacketSize;
		ep->num = epnum;
		ep->dir = epdir;
		ep->type = iousb_ep->edesc.bmAttributes & 3;

		// set the mps for the endpoint		
		if ( ep->dir & USB_ENDPOINT_IN ) {
			HW_Write16( dc->IoBase, MUSB_TXMAXP( epnum ) , ep->mps );		
		} else {
			HW_Write16( dc->IoBase, MUSB_RXMAXP( epnum ) , ep->mps );		
		}

		/* Allocate fifo memory for the endpoint, and associate allocated memory with said endpoint */
		
		fifosz_in_bytes = ep->mps;
#ifdef MUSB_DOUBLE_BUFFER_ENABLED
		fifosz_in_bytes <<= 1;
		double_buffer_flag = FIFOSZ_DOUBLE_BUFFER;
#endif 

		// allocate memory... allocator will round up to the nearest power of 2 and modify the second parameter
		ep->fifoblk = fifomem_alloc( dc->fifomem_hdl, &fifosz_in_bytes );
		if ( ep->fifoblk < 0 ) {
			musb_slogf(dc, _SLOG_ERROR, "%s: fifomem_alloc() failed to allocate fifo of size = %d", __func__, 2 * ep->mps );
			return ENOMEM;
		}

		// select the endpoint index
		HW_Write8( dc->IoBase, MUSB_INDEX, epnum );
		
		/* set the fifo size and address for the endpoint... The fifo size is 
  		* specified in 8 bytes block
 		* 8 bytes: fifosize = 0
		* 16 byte: fifosize = 1
		* etc... 
		*/
		v = double_buffer_flag | musb_log2( fifosz_in_bytes >> 3 );

		
		musb_slogf(dc, _SLOG_INFO, "%s: epnum = 0x%x mps = %d  fifoblk = %d FIFOSZREG = 0x%x ", __func__, ep->num, ep->mps, ep->fifoblk, v );
		
		if ( ep->dir & USB_ENDPOINT_IN ) {
			HW_Write8( dc->IoBase, MUSB_TXFIFOSZ, v );
			HW_Write16( dc->IoBase, MUSB_TXFIFOADD, ep->fifoblk );
		} else {
			HW_Write8( dc->IoBase, MUSB_RXFIFOSZ, v );
			HW_Write16( dc->IoBase, MUSB_RXFIFOADD, ep->fifoblk );
		}
		
		// This endpoint uses both virtual and physical addresses, because
		// a mixture of both DMA and PIO are used.  use BMAP so that urb->buffer
		// is a virtual address, and access physical address via urb->bufer_paddr;
		iousb_ep->flags = IOUSB_ENDPOINT_FLAGS_BMAP;
		
		// allocate dma channel if it is a bulk endpoint, 
		if ( ( dc->flags & DC_FLAG_USE_DMA ) && ( ep->type == USB_ATTRIB_BULK ) ) {
			ep->dma_chidx = dc->dma_funcs.channel_claim( dc->dma_hdl, ep );
			if ( ep->dma_chidx>= 0 ) {
				musb_slogf(dc, _SLOG_INFO, "%s: epnum = 0x%x uses dma channel index = %d ", __func__, ep->num, ep->dma_chidx );
			} else {
				musb_slogf(dc, _SLOG_WARNING, "%s: epnum = 0x%x has no more dma channels to use... using PIO ", __func__, ep->num);
			}
		 } else {
			 // don't use dma
			 ep->dma_chidx = -1;
		 }
	}
	
	// reset some state variables
	ep->urb = 0;
	ep->xfer_buffer = 0;
	ep->xfer_length = 0;
	ep->xfer_flags = 0;
	ep->req_xfer_len = 0;
	
	// reset the toggle
	if ( ep->dir & USB_ENDPOINT_IN ) {
		HW_Write16( dc->IoBase, MUSB_TXCSR( ep->num ), TXCSR_CLR_DATA_TOGGLE );
		if ( ep->dma_chidx != -1 ) {
			// The DMA enable bit is set in the transfer function because the transfer function need
			// to use PIO for sending the ZLP
			HW_Write16Or( dc->IoBase, MUSB_TXCSR( ep->num ), TXCSR_AUTOSET | TXCSR_DMA_REQ_TYPE1  );
		}
	} else {
		HW_Write16( dc->IoBase, MUSB_RXCSR( ep->num ), RXCSR_CLR_DATA_TOGGLE );
		if ( ep->dma_chidx != -1 ) {
			// we set the DMA enable here because we try to dma every transfer... short transfers trigger an endpoint interrtupt
			// and are handled via PIO
			HW_Write16Or( dc->IoBase, MUSB_RXCSR( ep->num ), RXCSR_AUTOCLEAR | RXCSR_DMA_REQ_TYPE1 | RXCSR_DMA_REQ_EN );
		}
	}
	
	return EOK;
}

_uint32 
musb_isoch_endpoint_enable( void *chdl, iousb_device_t *device, iousb_endpoint_t *iousb_ep )
{
	usbdc_device_t *udc = ( usbdc_device_t * ) chdl;
	dctrl_t * dc = ( dctrl_t * ) udc->dc_data;

	musb_slogf(dc, _SLOG_ERROR, "%s: Not Supported", __func__ );
	
	return EOK;
}

_uint32 
musb_endpoint_disable( void *chdl, iousb_endpoint_t *iousb_ep )
{
	usbdc_device_t *udc = ( usbdc_device_t * ) chdl;
	dctrl_t * dc = ( dctrl_t * ) udc->dc_data;
	ep_ctx_t				*ep = iousb_ep->user;

	iousb_ep->user = 0;
	
	if ( ep ) {
		if ( ep->dma_chidx != -1 ) {
			dc->dma_funcs.channel_release( dc->dma_hdl, ep->dma_chidx );
			ep->dma_chidx = -1;
		}
		
		fifomem_free(  dc->fifomem_hdl, ep->fifoblk );
	}
	
	
	return ( EOK );
}

_uint32 
musb_control_endpoint_disable( void *chdl, iousb_endpoint_t *iousb_ep )
{
	usbdc_device_t *udc = ( usbdc_device_t * ) chdl;
	dctrl_t * dc = ( dctrl_t * ) udc->dc_data;
	
	musb_slogf(dc, _SLOG_INFO, "%s: ep = 0x%x", __func__, iousb_ep->edesc.bEndpointAddress );
	iousb_ep->user = 0;
	
	return ( EOK );
}

_uint32 
musb_isoch_endpoint_disable( void *chdl, iousb_endpoint_t *iousb_ep )
{
	usbdc_device_t *udc = ( usbdc_device_t * ) chdl;
	dctrl_t * dc = ( dctrl_t * ) udc->dc_data;

	musb_slogf(dc, _SLOG_ERROR, "%s: Not Supported", __func__ );
	
	return EOK;
}
	
_uint32 
musb_control_transfer_abort( void *chdl, iousb_transfer_t *urb, iousb_endpoint_t *iousb_ep )
{
	usbdc_device_t *udc = ( usbdc_device_t * ) chdl;
	dctrl_t * dc = ( dctrl_t * ) udc->dc_data;
	ep_ctx_t 		*ep = iousb_ep->user;
	
	if ( ep == NULL ) {
		return EOK;
	}

	MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: ep = 0x%x  ep->num = %d ", __func__, iousb_ep->edesc.bEndpointAddress, ep->num);
	
	if( pthread_mutex_lock( &dc->mutex ) )
		musb_slogf(dc, _SLOG_ERROR, "%s: mutex lock failed ", __func__);

	
	// Null out the urb reference so that the completion function cannot complete
	// the urb just in case the abort fails to stop the hardware on time
	ep->urb = NULL;
	
	if ( ep->flags & EP_FLAG_DEFER_SETUP_PKT_CLEAR )  {
		// complete defered processing for NO DATAPHASE control transfer
		HW_Write16( dc->IoBase, MUSB_CSR0, CSR0_SERVICED_RXPKTRDY );
		ep->flags &= ~EP_FLAG_DEFER_SETUP_PKT_CLEAR;
	}
	
	flush_fifo( dc, ep );
	
	if( pthread_mutex_unlock( &dc->mutex ) )
		musb_slogf(dc, _SLOG_ERROR, "%s: mutex unlock failed ", __func__ );
	
	return EOK;
}

_uint32 
musb_control_transfer( void *chdl, iousb_transfer_t *urb, iousb_endpoint_t *iousb_ep, uint8_t *buffer, _uint32 length, _uint32 flags )
{
	usbdc_device_t 	*udc = ( usbdc_device_t * ) chdl;
	dctrl_t 		*dc = ( dctrl_t * ) udc->dc_data;
	ep_ctx_t 		*ep = iousb_ep->user;
	
	
	if( pthread_mutex_lock( &dc->mutex ) )
		musb_slogf(dc, _SLOG_ERROR, "%s: mutex lock failed ", __func__);

	MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: ep = 0x%x  ep->num = %d buffer = 0x%x len = %d flags = 0x%x ", __func__, iousb_ep->edesc.bEndpointAddress, ep->num, buffer,length,flags );
	
	ep->urb = urb;
	ep->xfer_buffer = (uint32_t)buffer;
	ep->xfer_length = length;
	ep->xfer_flags = flags;
	ep->bytes_xfered = 0;

	if ( flags & PIPE_FLAGS_TOKEN_STATUS ) {

		/* Complete the URB now because the hardware completes the status phase automatically and 
		 * it is very difficult ( if not impossible ) to to determine the state
		 * of the hardware once the interrupt fires because no actual status bits
		 * The HOST will take corrective action anyway if something goes wrong
		 */
		complete_urb( dc, ep, 0 );
		dc->ep0.ep0_idle_substate = EP0_IDLE_SUBSTATE_SETUP_PHASE;

		if ( ep->flags & EP_FLAG_DEFER_SETUP_PKT_CLEAR )  {
			// complete defered processing for NO DATAPHASE control transfer
			HW_Write16( dc->IoBase, MUSB_CSR0, CSR0_SERVICED_RXPKTRDY | CSR0_DATAEND );
			ep->flags &= ~EP_FLAG_DEFER_SETUP_PKT_CLEAR;
		}

		if ( ep->flags & EP_FLAG_DEFER_SETUP_PKT_PROCESSING ) {
			/* We have already received the next setup pkt, but we elected
			 * to defer processing until the previous status phase was complete
			 * Since we have already procesded the interrupt, need to call the 
			 * ep0_interrupt() hanlder manually
			 */
			MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: Start defered processing for endpoint 0",__func__);		
			ep->flags &= ~EP_FLAG_DEFER_SETUP_PKT_PROCESSING;
			process_ep0_interrupt( dc );
		}

	} else {
		// data phase
		
		if ( flags & PIPE_FLAGS_TOKEN_IN ) {
			dc->ep0.ep0_state = EP0_STATE_TXDATA;
			ep0_transmit( dc );
		} else {

			if ( ep->flags & EP_FLAG_DEFER_SETUP_PKT_CLEAR )  {
				// clear setup packet, and get ready to receive DATA-OUT dataphase
				HW_Write16( dc->IoBase, MUSB_CSR0, CSR0_SERVICED_RXPKTRDY );
				ep->flags &= ~EP_FLAG_DEFER_SETUP_PKT_CLEAR;
			}
			
			dc->ep0.ep0_state = EP0_STATE_RXDATA;
			ep0_receive( dc );
		}
	}

	if( pthread_mutex_unlock( &dc->mutex ) )
		musb_slogf(dc, _SLOG_ERROR, "%s: mutex unlock failed ", __func__ );
	
	return EOK;
}

	
_uint32 
musb_transfer_abort( void *chdl, iousb_transfer_t *urb, iousb_endpoint_t *iousb_ep )
{
	usbdc_device_t 	*udc = ( usbdc_device_t * ) chdl;
	dctrl_t 		*dc = ( dctrl_t * ) udc->dc_data;
	ep_ctx_t 		*ep = iousb_ep->user;
	
	if ( ep == NULL ) {
		return EOK;
	}
	
	if( pthread_mutex_lock( &dc->mutex ) )
		musb_slogf(dc, _SLOG_ERROR, "%s: mutex lock failed ", __func__);

	MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: ep = 0x%x  ep->num = %d ", __func__, iousb_ep->edesc.bEndpointAddress, ep->num);
	
	// Null out the urb reference so that the completion function cannot complete
	// the urb just in case the abort fails to stop the hardware on time
	ep->urb = NULL;
	
	if ( USE_DMA( ep ) ) {
		// stop transfer is progress
		dc->dma_funcs.transfer_abort( dc->dma_hdl, ep->dma_chidx );
	}
	
	flush_fifo( dc, ep );
	
	if( pthread_mutex_unlock( &dc->mutex ) )
		musb_slogf(dc, _SLOG_ERROR, "%s: mutex unlock failed ", __func__ );
	
	return EOK;
}

_uint32 
musb_transfer( void *chdl, iousb_transfer_t *urb, iousb_endpoint_t *iousb_ep, uint8_t *buffer, _uint32 length, _uint32 flags )
{
	usbdc_device_t 	*udc = ( usbdc_device_t * ) chdl;
	dctrl_t 		*dc = ( dctrl_t * ) udc->dc_data;
	ep_ctx_t 		*ep = iousb_ep->user;
	
	MUSB_SLOGF_DBG(dc, _SLOG_DEBUG1, "%s: ep = 0x%x  ep->num = %d buffer = 0x%x len = %d flags = 0x%x ", __func__, iousb_ep->edesc.bEndpointAddress, ep->num, buffer,length,flags );
	
	if( pthread_mutex_lock( &dc->mutex ) )
		musb_slogf(dc, _SLOG_ERROR, "%s: mutex lock failed ", __func__);

	
	
	ep->urb = urb;
	ep->xfer_buffer = (uint32_t)buffer;
	ep->xfer_buffer_paddr = (uint32_t) urb->buffer_paddr;
	ep->xfer_length = length;
	ep->xfer_flags = flags;
	ep->bytes_xfered = 0;
	
	if ( flags & PIPE_FLAGS_TOKEN_IN ) {
		ep_transmit( dc, ep );
	}    
	else {
		ep_receive( dc, ep );
	}
	
	if( pthread_mutex_unlock( &dc->mutex ) )
		musb_slogf(dc, _SLOG_ERROR, "%s: mutex unlock failed ", __func__ );
	
	return EOK;
}

_uint32 
musb_isoch_transfer_abort( void *chdl, iousb_transfer_t *urb, iousb_endpoint_t *iousb_ep )
{
	usbdc_device_t *udc = ( usbdc_device_t * ) chdl;
	dctrl_t * dc = ( dctrl_t * ) udc->dc_data;

	musb_slogf(dc, _SLOG_ERROR, "%s: Not Supported", __func__ );
	
	return ENOTSUP;	
}


_uint32 
musb_isoch_transfer( void *chdl, iousb_transfer_t *urb, iousb_endpoint_t *iousb_ep, uint8_t *buffer, _uint32 length, _uint32 flags )
{
	usbdc_device_t *udc = ( usbdc_device_t * ) chdl;
	dctrl_t * dc = ( dctrl_t * ) udc->dc_data;

	musb_slogf(dc, _SLOG_ERROR, "%s: Not Supported", __func__ );
	
	return ENOTSUP;
}


int
musb_process_args( dctrl_t *dc, char *args )
{

	int opt;
	char *value;
	char *c;
	int len;
	
	static char *driver_opts[] = {
		"verbose", 
		"ser",
		"linkup",
		"linkdown",
		"forcefs",
		"nodma",
		"no_cfg_pmic",
		"no_session_poll",
		"smartclock",
		"inherit_cfg",
		"USBPHY_ANA_CONFIG1",
		"USBPHY_ANA_CONFIG2",
		NULL
	};
	
	dc->serial_string = NULL;
	
	if( !args ) 
		return EOK;    
	
	// convert args
	len = strlen( args );
	while ( ( c = strchr( args, ':' ) ) ) {
		if ( c - args > len )
			break;
			*c = ',';
	}
		
	while( *args != '\0' ) {
		if( ( opt = getsubopt( &args, driver_opts, &value ) ) != -1 ) {
			switch ( opt ) {
				case 0 :
					if ( value )
						dc->verbosity = strtoul( value, 0, 0 );
					else 
						dc->verbosity = 5;
					continue;
				case 1 :     // this arg should move up for know we build a proper string desc
					if ( value ) {
						uint8_t  slen;
						uint32_t x;
						slen = min(strlen( value ), 127 ); // max 8bit length each char requires 2 bytes for encoding plus 2 byte header.
						dc->udc->serial_string = calloc( 1, 3 + 2 * slen );
						dc->udc->serial_string[0] = 2 + slen *2; // string header is 2 bytes
						dc->udc->serial_string[1] = USB_DESC_STRING;
						for ( x = 1 ; x < slen + 1 ; x ++ ) {
							dc->udc->serial_string[x * 2] = value[x - 1];
						}
					}
					continue;
				case 2:
					// "linkup"
					dc->flags |= DC_FLAG_SOFT_CONNECT;
				
					continue;
				
				case 3:
					// "linkdown"
					dc->flags &= ~DC_FLAG_SOFT_CONNECT;
					continue;
					
				case 4:
					// "forcefs"
					dc->flags |= DC_FLAG_FORCE_FS;
					continue;
					
				case 5:
					// "nodma"
					dc->flags &= ~DC_FLAG_USE_DMA;
					break;
				case 6:
					dc->flags &= ~DC_FLAG_CFG_PMIC;
					break;
				case 7:
					dc->flags &= ~DC_FLAG_SESSION_POLL;
					break;
				case 8:
					dc->flags |= DC_FLAG_SMART_CLOCK;
					break;
				case 9:
					dc->flags |= DC_FLAG_INHERIT_CFG;
					break;
				case 10:
					// USBPHY_ANA_CONFIG1
					dc->phy_tuning[USBPHY_ANA_CONFIG1] = strtoul( value, NULL, 16 );
					break;
				case 11:
					// USBPHY_ANA_CONFIG2
					dc->phy_tuning[USBPHY_ANA_CONFIG2] = strtoul( value, NULL, 16 );
					break;
				default :
					break;
			}
		}
	}
	
	return EOK;
}

static void 
default_config_set( usbdc_device_t *udc )
{
	dctrl_t * dc = ( dctrl_t * ) udc->dc_data;
	
	dc->verbosity = _SLOG_WARNING;
	dc->flags     =  DC_FLAG_USE_DMA | DC_FLAG_CFG_PMIC | DC_FLAG_SESSION_POLL;
	memset ( dc->phy_tuning, 0, sizeof( dc->phy_tuning ) );
    
	udc->hw_ctrl.cname = "dwcotg";
	udc->hw_ctrl.max_transfer_size = 0x10000; 
	udc->hw_ctrl.max_unaligned_xfer = 0x10000; 
	udc->hw_ctrl.buff_alignment_mask = 0x1;
	udc->hw_ctrl.capabilities 	= DC_CAP_FULL_SPEED | DC_CAP_HIGH_SPEED | DC_CAP_TEST_MODES_SUPPORTED;
	
}

static int reset_controller( dctrl_t * dc ) 
{
    int         retval;
    
    retval = musb_custom_reset( dc );
    
    
    return retval;
}

static int 
chip_config ( dctrl_t * dc ) 
{
	int err;
	
	err = reset_controller( dc );
	if ( err != EOK ) {
		goto fail;
	}
	
	if ( dc->flags & DC_FLAG_FORCE_FS ) {
		HW_Write8And( dc->IoBase, MUSB_POWER, ~POWER_HS_ENABLE );
	}

#ifndef EXTERNAL_INTERRUPT_PROCESSING
	// enable main usb interupts
	HW_Write8( dc->IoBase, MUSB_INTRUSBE, 
				INTRUSB_SUSPEND | INTRUSB_RESUME  | INTRUSB_RESET | INTRUSB_DISCON );
#endif

	return EOK;	

fail:	
	return err;
}



uint32_t
musb_init( usbdc_device_t *udc, io_usbdc_self_t *udc_self, char *args )
{
	dctrl_t * dc;
	pthread_mutexattr_t mattr;    
	int err;
	char *args_copy;
    
    /* allocate device ctx */
	dc = udc->dc_data = calloc( 1, sizeof( dctrl_t ) );
	if ( !dc ) {
		musb_slogf( dc, _SLOG_ERROR, "%s calloc failed",__func__);
		err = ENOMEM;
		goto error;
	}
	dc->udc = udc;
    
	/* Get SoC specific parameters for the musbmhdrc controller */
	musb_cfg_get( &dc->musbmhdrc_cfg );
	
    /* set default driver configurations */
	default_config_set( udc );
	
    /* parse command line arguments to override default configs */
    
    args_copy = strdup( args );
    if ( args_copy == NULL ) {
		musb_slogf( dc, _SLOG_ERROR, "%s strdup failed",__func__);
		err = ENOMEM;
		goto error2;
    }
    
	err = musb_process_args(dc, args);
	if ( err ) {
		musb_slogf( dc, _SLOG_ERROR, "%s couldn't parse command line args",__func__);
		goto error3;        
	}
    
	err = musb_custom_init1( dc, args_copy );
	if ( err ) {
		musb_slogf( dc, _SLOG_ERROR, "%s musb_custom_init1() failed err = %d",__func__, err);
		goto error3;        
	}
	
	// no longer need args copy, because it has alreay been processed by custom_int1
	free( args_copy );
	args_copy = NULL;
	
	// map io
	dc->IoBase =  mmap_device_memory( NULL, 
						MUSB_SIZE,  
						PROT_READ | PROT_WRITE | PROT_NOCACHE, 
						MAP_SHARED | MAP_PHYS,
						PCI_MEM_ADDR( udc->hw_ctrl.pci_inf->CpuBaseAddress[0] ) );
	if ( dc->IoBase == MAP_FAILED ) {
		musb_slogf( dc, _SLOG_ERROR, "%s mmap failed",__func__);
		err = ENOMEM;
		goto error4;
	}

	// create the driver mutex    
	pthread_mutexattr_init( &mattr );
	pthread_mutexattr_setrecursive( &mattr, PTHREAD_RECURSIVE_ENABLE );
	if( pthread_mutex_init( &dc->mutex, &mattr ) == -1 ) {
		musb_slogf( dc, _SLOG_ERROR, "%s could not create mutex",__func__);
		err = ENOMEM;
		goto error5;
	}

	if ( dc->flags & DC_FLAG_USE_DMA ) {
		// map the dma funcs for this SoC
		musb_dma_map( &dc->dma_funcs );
	
		dc->dma_hdl = dc->dma_funcs.init( dc ); 
		if ( dc->dma_hdl == NULL ) {
			musb_slogf( dc, _SLOG_ERROR, "%s could not initialize the DMA",__func__);
			err = -1;
			goto error6;
		}
	}

	// Initialize FIFO memory allocator
	dc->fifomem_hdl = fifomem_init( dc->musbmhdrc_cfg.fiforam_size );
	if ( dc->fifomem_hdl == NULL ) {
		musb_slogf( dc, _SLOG_ERROR, "%s failed to initialize fifo memory allocator",__func__);
		goto error7;
	}

	// setup usb controller
	err = chip_config( dc );
	if ( err ) {
		musb_slogf( dc, _SLOG_ERROR, "%s could not init the controller",__func__);
		goto error8;
	}
	
	err = musb_custom_init2( dc );
	if ( err ) {
		musb_slogf( dc, _SLOG_ERROR, "%s musb_custom_init2() failed err = %d",__func__, err);
		goto error8;        
	}
	
	udc->usbdc_self->usbdc_set_device_speed( udc, IOUSB_DEVICE_FULL_SPEED );  
	
	
	// set Soft-Connect state	
	if ( dc->flags & DC_FLAG_SOFT_CONNECT ) {
		HW_Write8Or( dc->IoBase, MUSB_POWER, POWER_SOFT_CONN );
	} else {
		HW_Write8And( dc->IoBase, MUSB_POWER, ~POWER_SOFT_CONN );
	}
	
	return EOK;

error8:
	fifomem_fini( dc->fifomem_hdl ) ;
error7:
	if ( dc->flags & DC_FLAG_USE_DMA ) {
		dc->dma_funcs.fini( dc->dma_hdl );
	}
error6:
	pthread_mutex_destroy( &dc->mutex );
error5:
	munmap( dc->IoBase, MUSB_SIZE );
error4:
	musb_custom_fini1( dc );
error3:
	if ( args_copy ) 
		free( args_copy );
error2:
	free(dc);  
error:    
	return err;
}

uint32_t
musb_start( usbdc_device_t *udc )
{
	return EOK;
}

uint32_t
musb_stop( usbdc_device_t *udc )
{
	return EOK;
}

uint32_t
musb_shutdown( usbdc_device_t *udc )
{
	dctrl_t * dc = ( dctrl_t * ) udc->dc_data;
	
	// force disconnect
	HW_Write8And( dc->IoBase, MUSB_POWER, ~POWER_SOFT_CONN );

	// free resources
	fifomem_fini( dc->fifomem_hdl ) ;
	if ( dc->flags & DC_FLAG_USE_DMA ) {
		dc->dma_funcs.fini( dc->dma_hdl );	
	}
	pthread_mutex_destroy( &dc->mutex );
	munmap( dc->IoBase, MUSB_SIZE);
	musb_custom_fini2( dc );
	musb_custom_fini1( dc );
	free(dc);  
	
	return EOK;
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.6.0/trunk/hardware/devu/dc/musbmhdrc/musb.c $ $Rev: 735878 $")
#endif
