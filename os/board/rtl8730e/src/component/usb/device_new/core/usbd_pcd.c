/**
  ******************************************************************************
  * @file    usbd_pcd.c
  * @author  Realsil WLAN5 Team
  * @brief   This file provides the functionalities of USB PCD layer
  ******************************************************************************
  * @attention
  *
  * This module is a confidential and proprietary property of RealTek and
  * possession or use of this module requires written permission of RealTek.
  *
  * Copyright(c) 2020, Realtek Semiconductor Corporation. All rights reserved.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usb_hal.h"
#include "usbd_core.h"
#include "usbd_pcd.h"
#include "usbd_hal.h"

/* Private defines -----------------------------------------------------------*/

/* Enable in token prediction debug */
#define IN_TOKEN_PREDICT_DEBUG_EN              0U

/* Private types -------------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

static u8 usbd_pcd_interrupt_init(usbd_pcd_t *pcd);
static u8 usbd_pcd_interrupt_deinit(usbd_pcd_t *pcd);
static void usbd_pcd_isr_task(void *data);
static void usbd_pcd_irq_handler(void);
static void usbd_pcd_handle_interrupt(usbd_pcd_t *pcd);
static void usbd_pcd_handle_reset_interrupt(usbd_pcd_t *pcd);
static void usbd_pcd_handle_enum_done_interrupt(usbd_pcd_t *pcd);
static void usbd_pcd_handle_suspend_interrupt(usbd_pcd_t *pcd);
static void usbd_pcd_handle_ep_out_setup_packet_interrupt(usbd_pcd_t *pcd, u32 ep_num);
static void usbd_pcd_handle_ep_out_transfer_complete_interrupt(usbd_pcd_t *pcd, u32 ep_num);
static void usbd_pcd_handle_out_ep_interrupt(usbd_pcd_t *pcd);
static void usbd_pcd_handle_in_ep_disabled_interrupt(usbd_pcd_t *pcd, u32 ep_num);
static void usbd_pcd_handle_in_ep_interrupt(usbd_pcd_t *pcd);
static void usbd_pcd_handle_rx_fifo_non_empty_interrupt(usbd_pcd_t *pcd);
static void usbd_pcd_get_in_ep_sequence_from_in_token_queue(usbd_pcd_t *pcd);
static u8 usbd_pcd_handle_ep_np_tx_fifo_empty_interrupt(usbd_pcd_t *pcd, u8 ep_num);
static void usbd_pcd_handle_np_tx_fifo_empty_interrupt(usbd_pcd_t *pcd);
static void usbd_pcd_handle_ep_mismatch_interrupt(usbd_pcd_t *pcd);
static void usbd_pcd_handle_in_nak_effective(usbd_pcd_t *pcd);
static void usbd_pcd_handle_wakeup_interrupt(usbd_pcd_t *pcd);
static void usbd_pcd_handle_sof_interrupt(usbd_pcd_t *pcd);
static void usbd_pcd_handle_srq_interrupt(usbd_pcd_t *pcd);
static void usbd_pcd_handle_otg_interrupt(usbd_pcd_t *pcd);
static void usbd_pcd_handle_interrupt(usbd_pcd_t *pcd);

/* Private variables ---------------------------------------------------------*/

/* PCD handler */
USB_BSS_SECTION
static usbd_pcd_t usbd_pcd;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize the PCD interrupt.
  * @param  pcd: PCD handle
  * @retval HAL status
  */
USB_TEXT_SECTION
static u8 usbd_pcd_interrupt_init(usbd_pcd_t *pcd)
{
	int ret;

	if (pcd->isr_initialized) {
		usbd_pcd_interrupt_deinit(pcd);
	}

	rtw_init_sema(&pcd->isr_sema, 0);

	ret = rtw_create_task(&pcd->isr_task,
						  "usbd_isr_task",
						  USBD_ISR_THREAD_STACK_SIZE,
						  pcd->config.isr_priority,
						  usbd_pcd_isr_task,
						  (void *)pcd);
	if (ret != 1) {
		return HAL_ERR_UNKNOWN;
	}

	usb_hal_register_irq_handler((usb_irq_fun_t)usbd_pcd_irq_handler);
	usb_hal_enable_interrupt();

	pcd->isr_initialized = 1;

	return HAL_OK;
}

/**
  * @brief  Deinitialize the PCD interrupt.
  * @param  pcd: PCD handle
  * @retval HAL status
  */
USB_TEXT_SECTION
static u8 usbd_pcd_interrupt_deinit(usbd_pcd_t *pcd)
{
	if (pcd->isr_initialized) {
		usb_hal_disable_interrupt();
		usb_hal_unregister_irq_handler();
		if (pcd->isr_task.task > 0) {
			rtw_delete_task(&pcd->isr_task);
		}
		rtw_free_sema(&pcd->isr_sema);
		pcd->isr_initialized = 0;
	}
	return HAL_OK;
}

/**
  * @brief  PCD ISR task
  * @param  data: PCD handle
  * @retval void
  */
USB_TEXT_SECTION
static void usbd_pcd_isr_task(void *data)
{
	usbd_pcd_t *pcd = (usbd_pcd_t *)data;
	u32 gintsts;

	for (;;) {
		rtw_down_sema(&pcd->isr_sema);

		if (pcd->isr_task.blocked) {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_WARN, "USBD ISR task killed\n");
			break;
		}

		gintsts = usb_hal_read_interrupts();

		usbd_pcd_handle_interrupt(pcd);

		//rtw_enter_critical(NULL, NULL);
		usb_hal_enable_interrupt();
		//rtw_exit_critical(NULL, NULL);

		if ((gintsts != 0U) && (usb_hal_get_otg_mode() == USB_OTG_MODE_DEVICE)) {
			if ((gintsts & USB_OTG_GINTSTS_USBSUSP) != 0) {
				if (pcd->dev->dev_attach_status == USBD_ATTACH_STATUS_ATTACHED) {
					pcd->dev->dev_attach_status = USBD_ATTACH_STATUS_DETACHED;
				}
			} else {
				if ((pcd->dev->dev_attach_status != USBD_ATTACH_STATUS_ATTACHED) && ((gintsts & USB_OTG_GINTSTS_ESUSP) == 0)) {
					pcd->dev->dev_attach_status = USBD_ATTACH_STATUS_ATTACHED;
				}
			}
		}
	}

	rtw_thread_exit();
}

/**
  * @brief  USB device IRQ handler
  * @param  None
  * @retval void
  */
USB_TEXT_SECTION
static void usbd_pcd_irq_handler(void)
{
	usbd_pcd_t *pcd = &usbd_pcd;
	usb_hal_disable_interrupt();
	rtw_up_sema_from_isr(&pcd->isr_sema);
}

/**
  * @brief  Handle PCD reset interrupt.
  * @param  pcd: PCD handle
  * @retval void
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_reset_interrupt(usbd_pcd_t *pcd)
{
	u32 i;

	USB_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;

	usb_hal_flush_tx_fifo(0x10U);

	for (i = 0U; i < USB_MAX_ENDPOINTS; i++) {
		USB_INEP(i)->DIEPINT = 0xFFFFU; // 0xFB7FU;
		USB_INEP(i)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
		USB_OUTEP(i)->DOEPINT = 0xFFFFU; // 0xFB7FU;
		USB_OUTEP(i)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
	}

	usbd_hal_reset_in_token_queue(pcd);

	USB_DEVICE->DAINTMSK |= 0x10001U;

	USB_DEVICE->DOEPMSK |= USB_OTG_DOEPMSK_STUPM |
						   USB_OTG_DOEPMSK_XFRCM |
						   USB_OTG_DOEPMSK_EPDM |
						   USB_OTG_DOEPMSK_OTEPSPRM;

	USB_DEVICE->DIEPMSK |= USB_OTG_DIEPMSK_TOM |
						   USB_OTG_DIEPMSK_XFRCM |
						   USB_OTG_DIEPMSK_EPDM |
						   USB_OTG_DIEPMSK_INEPNMM;

	if ((pcd->config.ext_intr_en & USBD_ITTXFE_INTR) && (!pcd->ded_tx_fifo_en) && (!pcd->config.dma_enable)) { //share FIFO & no dma
		USB_DEVICE->DIEPMSK |= USB_OTG_DIEPMSK_ITTXFEMSK;
	}

	/* Set Default Address to 0 */
	USB_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD;

	/* setup EP0 to receive SETUP packets */
	(void)usbd_hal_ep0_out_start(pcd);

}

/**
  * @brief  Handle PCD enum done interrupt.
  * @param  pcd: PCD handle
  * @retval void
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_enum_done_interrupt(usbd_pcd_t *pcd)
{
	(void)usbd_hal_ep0_setup_activate(pcd);

	pcd->config.speed = usbd_hal_get_device_speed(pcd);

	/* Set USB Turnaround time */
	(void)usbd_hal_set_turnaround_time(pcd);

	usb_os_spinunlock(pcd->lock);
	usbd_core_set_speed(pcd->dev, (usb_speed_type_t)pcd->config.speed);

	/* Reset Device */
	usbd_core_reset(pcd->dev);
	usb_os_spinlock(pcd->lock);
}

/**
  * @brief  Handle PCD suspend interrupt.
  * @param  pcd: PCD handle
  * @retval void
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_suspend_interrupt(usbd_pcd_t *pcd)
{
	if ((USB_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS) == USB_OTG_DSTS_SUSPSTS) {
		usb_os_spinunlock(pcd->lock);
		usbd_core_suspend(pcd->dev);
		usb_os_spinlock(pcd->lock);
	}
}

/**
  * @brief  process EP OUT setup packet received interrupt.
  * @param  pcd: PCD handle
  * @param  ep_num: Endpoint number
  * @retval HAL status
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_ep_out_setup_packet_interrupt(usbd_pcd_t *pcd, u32 ep_num)
{
	u32 gSNPSiD = USB_GLOBAL->GSNPSID;
	u32 DoepintReg = USB_OUTEP(ep_num)->DOEPINT;

	if (pcd->config.dma_enable) {
		/* StupPktRcvd = 1 pending setup packet int */
		if ((gSNPSiD > USB_OTG_CORE_ID_300A) &&
			((DoepintReg & USB_OTG_DOEPINT_STPKTRX) == USB_OTG_DOEPINT_STPKTRX)) {
			USB_PCD_CLEAR_OUT_EP_INTR(ep_num, USB_OTG_DOEPINT_STPKTRX);
		}
		if (pcd->setup != NULL) {
			DCache_Invalidate((u32)pcd->setup, USBD_SETUP_PACKET_BUF_LEN);
		}
	} else {
		if ((gSNPSiD == USB_OTG_CORE_ID_310A) &&
			((DoepintReg & USB_OTG_DOEPINT_STPKTRX) == USB_OTG_DOEPINT_STPKTRX)) {
			USB_PCD_CLEAR_OUT_EP_INTR(ep_num, USB_OTG_DOEPINT_STPKTRX);
		}
	}

	/* Inform the upper layer that a setup packet is available */
	usb_os_spinunlock(pcd->lock);
	usbd_core_setup_stage(pcd->dev, (u8 *) pcd->setup);
	usb_os_spinlock(pcd->lock);
}

/**
  * @brief  process EP OUT transfer complete interrupt.
  * @param  pcd: PCD handle
  * @param  ep_num: Endpoint number
  * @retval HAL status
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_ep_out_transfer_complete_interrupt(usbd_pcd_t *pcd, u32 ep_num)
{
	u32 pktcnt;
	u32 xfer_size;
	u32 gSNPSiD = USB_GLOBAL->GSNPSID;
	u32 DoepintReg = USB_OUTEP(ep_num)->DOEPINT;
	usbd_pcd_ep_t *ep = &pcd->out_ep[ep_num];

	if (pcd->config.dma_enable) {
		if ((DoepintReg & USB_OTG_DOEPINT_OTEPSPR) == USB_OTG_DOEPINT_OTEPSPR) { /* Class E */
			USB_PCD_CLEAR_OUT_EP_INTR(ep_num, USB_OTG_DOEPINT_OTEPSPR);
		} else if ((DoepintReg & (USB_OTG_DOEPINT_STUP | USB_OTG_DOEPINT_OTEPSPR)) == 0U) {
			/* StupPktRcvd = 1 this is a setup packet */
			if ((gSNPSiD > USB_OTG_CORE_ID_300A) &&
				((DoepintReg & USB_OTG_DOEPINT_STPKTRX) == USB_OTG_DOEPINT_STPKTRX)) {
				USB_PCD_CLEAR_OUT_EP_INTR(ep_num, USB_OTG_DOEPINT_STPKTRX);
			} else {
				/* out data packet received */
				if (ep_num == 0) {
					xfer_size = USB_OUTEP(ep_num)->DOEPTSIZ & USB_OTG_DOEPTSIZ0_XFRSIZ;

					ep->xfer_count = ep->max_packet_len - xfer_size;
				} else {
					xfer_size = USB_OUTEP(ep_num)->DOEPTSIZ & USB_OTG_DOEPTSIZ_XFRSIZ;

					pktcnt = (ep->xfer_len + ep->max_packet_len - 1U) / ep->max_packet_len;

					ep->xfer_count = ep->max_packet_len * pktcnt - xfer_size;
				}

				if ((ep->xfer_count != 0U) && (ep->xfer_buff != NULL)) {
					DCache_Invalidate((u32)ep->xfer_buff, ep->xfer_count);
				}
 
				if (ep->xfer_buff) {
					ep->xfer_buff += ep->xfer_count;/*For control OUT xfer*/
				}

				usb_os_spinunlock(pcd->lock);
				usbd_core_data_out_stage(pcd->dev, (u8)ep_num, pcd->out_ep[ep_num].xfer_buff);
				usb_os_spinlock(pcd->lock);

				if ((ep_num == 0U) && (pcd->out_ep[ep_num].xfer_len == 0U)) {
					/* this is ZLP, so prepare EP0 for next setup */
					(void)usbd_hal_ep0_out_start(pcd);
				}
			}
		} else {
			/* ... */
		}
	} else {
		if (gSNPSiD == USB_OTG_CORE_ID_310A) {
			/* StupPktRcvd = 1 this is a setup packet */
			if ((DoepintReg & USB_OTG_DOEPINT_STPKTRX) == USB_OTG_DOEPINT_STPKTRX) {
				USB_PCD_CLEAR_OUT_EP_INTR(ep_num, USB_OTG_DOEPINT_STPKTRX);
			} else {
				if ((DoepintReg & USB_OTG_DOEPINT_OTEPSPR) == USB_OTG_DOEPINT_OTEPSPR) {
					USB_PCD_CLEAR_OUT_EP_INTR(ep_num, USB_OTG_DOEPINT_OTEPSPR);
				}
				usb_os_spinunlock(pcd->lock);
				usbd_core_data_out_stage(pcd->dev, (u8)ep_num, pcd->out_ep[ep_num].xfer_buff);
				usb_os_spinlock(pcd->lock);
			}
		} else {
			usb_os_spinunlock(pcd->lock);
			usbd_core_data_out_stage(pcd->dev, (u8)ep_num, pcd->out_ep[ep_num].xfer_buff);
			usb_os_spinlock(pcd->lock);
		}
	}
}

/**
  * @brief  Handle PCD OUT EP interrupt.
  * @param  pcd: PCD handle
  * @retval void
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_out_ep_interrupt(usbd_pcd_t *pcd)
{
	u32 ep_intr;
	u32 epint;
	u32 ep_num = 0U;

	/* Read in the device interrupt bits */
	ep_intr = usbd_hal_read_all_out_ep_interrupts(pcd);
	while ((ep_intr != 0U) && (ep_num < USB_MAX_ENDPOINTS)) {
		if ((ep_intr & 0x1U) != 0U) {
			epint = usbd_hal_read_out_ep_interrupts(pcd, (u8)ep_num);

			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "OUT EP%d 0x%08X\n", ep_num, epint);

			if ((epint & USB_OTG_DOEPINT_XFRC) == USB_OTG_DOEPINT_XFRC) {
				DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "OUT EP%d XFRC\n", ep_num);
				USB_PCD_CLEAR_OUT_EP_INTR(ep_num, USB_OTG_DOEPINT_XFRC);
				usbd_pcd_handle_ep_out_transfer_complete_interrupt(pcd, ep_num);
			}

			if ((epint & USB_OTG_DOEPINT_STUP) == USB_OTG_DOEPINT_STUP) {
				DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "OUT EP%d STUP\n", ep_num);
				/* Class B setup phase done for previous decoded setup */
				usbd_pcd_handle_ep_out_setup_packet_interrupt(pcd, ep_num);
				USB_PCD_CLEAR_OUT_EP_INTR(ep_num, USB_OTG_DOEPINT_STUP);
			}

			if ((epint & USB_OTG_DOEPINT_OTEPDIS) == USB_OTG_DOEPINT_OTEPDIS) {
				DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "OUT EP%d DIS\n", ep_num);
				USB_PCD_CLEAR_OUT_EP_INTR(ep_num, USB_OTG_DOEPINT_OTEPDIS);
			}

			/* Clear Status Phase Received interrupt */
			if ((epint & USB_OTG_DOEPINT_OTEPSPR) == USB_OTG_DOEPINT_OTEPSPR) {
				DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "OUT EP%d SPR\n", ep_num);
				if (pcd->config.dma_enable == 1U) {
					(void)usbd_hal_ep0_out_start(pcd);
				}
				USB_PCD_CLEAR_OUT_EP_INTR(ep_num, USB_OTG_DOEPINT_OTEPSPR);
			}

			/* Clear OUT NAK interrupt */
			if ((epint & USB_OTG_DOEPINT_NAK) == USB_OTG_DOEPINT_NAK) {
				DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "OUT EP%d NAK\n", ep_num);
				USB_PCD_CLEAR_OUT_EP_INTR(ep_num, USB_OTG_DOEPINT_NAK);
			}
		}
		ep_num++;
		ep_intr >>= 1U;
	}
}

/**
  * @brief  Handle PCD IN EP disable interrupt.
  * @param  pcd: PCD handle
  * @param  ep_num: Endpoint number
  * @retval void
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_in_ep_disabled_interrupt(usbd_pcd_t *pcd, u32 ep_num)
{
	u32 i;
	u32 reg;
	u32 pktcnt_mask;
	u32 dma = pcd->config.dma_enable;
	usbd_pcd_ep_t *pep = &pcd->in_ep[ep_num];

	if ((pcd->start_predict == 0) || (pep->is_ptx)) {   // period
		// TODO: Clear the Global IN NP NAK and restart transfer
		return;
	}

	if (pcd->start_predict > 2)  {
		pcd->start_predict--;
		return;
	}

	pcd->start_predict--;

	if (pcd->start_predict == 1)  { // All NP IN Ep's disabled now
		if (dma == 1) {
			//predict nextep
			usbd_pcd_get_in_ep_sequence_from_in_token_queue(pcd);

			/* Update all active IN EP's NextEP field based of nextep_seq[] */
			for (i = 0; i < USB_MAX_ENDPOINTS; i++) {
				if (pcd->nextep_seq[i] != 0xff) {   //Active NP IN EP
					reg = USB_INEP(i)->DIEPCTL;
					reg &= ~USB_OTG_DIEPCTL_NEXTEP_Msk;
					reg |= pcd->nextep_seq[i] << USB_OTG_DIEPCTL_NEXTEP_Pos;
					USB_INEP(i)->DIEPCTL = reg;
				}
			}
		}

		/* Flush Shared NP TxFIFO */
		usb_hal_flush_tx_fifo(0x00);

		/*
			Not ZLP
			loop to report the error status to all active ep(NP IN Ep)
		*/
		usb_os_spinunlock(pcd->lock);
		for (i = 0; i < USB_MAX_ENDPOINTS; i++) {
			pep = &pcd->in_ep[i];
			reg = USB_INEP(i)->DIEPTSIZ;
			//EP0: PktCnt 20:19,XferSize 6:0
			//EPN: PktCnt 28:19,XferSize 18:0
			if (i == 0) {
				pktcnt_mask = USB_OTG_DIEPTSIZ0_PKTCNT;
			} else {
				pktcnt_mask = USB_OTG_DIEPTSIZ_PKTCNT;
			}
			if ((!(pep->is_zlp)) && (reg & pktcnt_mask) && (!(pep->is_ptx)) && (pep->is_initialized)) {
				DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "ip=%d report error\n", i);
				usbd_core_data_in_stage(pcd->dev, (u8)i, NULL, HAL_TIMEOUT);
			}
		}
		usb_os_spinlock(pcd->lock);

		/*
			ZLP ,Restart transfers
			re enable the EP, the ZLP transmit will continue and finish
		*/
		i = ((dma == 1) ? (pcd->first_in_nextep_seq) : (0));
		do {
			pep = &pcd->in_ep[i];
			reg = USB_INEP(i)->DIEPTSIZ;
			//EP0: PktCnt 20:19,XferSize 6:0
			//EPN: PktCnt 28:19,XferSize 18:0
			if (i == 0) {
				pktcnt_mask = USB_OTG_DIEPTSIZ0_PKTCNT;
			} else {
				pktcnt_mask = USB_OTG_DIEPTSIZ_PKTCNT;
			}
			//DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE,"for zlp ip=%d/zlp=%d/reg=0x%08x\n",i,pep->is_zlp,USB_INEP(i)->DIEPTSIZ);
			if ((pep->is_zlp) && (reg & pktcnt_mask) && (!(pep->is_ptx)) && (pep->is_initialized)) {
				DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "re enable ep%d\n", i);
				USB_INEP(i)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
			}

			if (dma == 1) {
				/*
					dma mode, loop all next_seq ep
				*/
				i = pcd->nextep_seq[i];
				if (i == pcd->first_in_nextep_seq) {
					break;
				}
			} else {
				/*
					slave mode, loop all ep
				*/
				if (++i >= USB_MAX_ENDPOINTS) {
					break;
				}
			}
		} while (1);

		/* Clear the global non-periodic IN NAK handshake */
		USB_DEVICE->DCTL |= USB_OTG_DCTL_CGINAK;
		/* Unmask EP Mismatch interrupt */
		USB_GLOBAL->GINTMSK |= USB_OTG_GINTMSK_EPMISM;

		pcd->start_predict = 0;
	}
}

/**
  * @brief  Write TxFIFO when EP TxFIFO emptry interrupt occurs
  * @param  pcd: PCD handle
  * @param  ep_num: Endpoint number
  * @retval HAL status
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_in_ep_tx_fifo_empty_interrupt(usbd_pcd_t *pcd, u32 ep_num)
{
	usbd_pcd_ep_t *ep;
	u32 len;
	u32 len32b;
	u32 msk;

	ep = &pcd->in_ep[ep_num];

	if (ep->xfer_count > ep->xfer_len) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "IN EP%d xfer error\n", ep_num);
		return;
	}

	len = ep->xfer_len - ep->xfer_count;

	if (len > ep->max_packet_len) {
		len = ep->max_packet_len;
	}

	len32b = (len + 3U) / 4U;

	while (((USB_INEP(ep_num)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) >= len32b) &&
		   (ep->xfer_count < ep->xfer_len) && (ep->xfer_len != 0U)) {
		/* Write the FIFO */
		len = ep->xfer_len - ep->xfer_count;

		if (len > ep->max_packet_len) {
			len = ep->max_packet_len;
		}

		len32b = (len + 3U) / 4U;

		if (!pcd->config.dma_enable) {
			usb_hal_write_packet(ep->xfer_buff, (u8)ep_num, (u16)len);
		}

		ep->xfer_buff  += len;
		ep->xfer_count += len;
	}

	if (ep->xfer_len <= ep->xfer_count) {
		msk = (0x1UL << (ep_num & EP_ADDR_MSK));
		USB_DEVICE->DIEPEMPMSK &= ~msk;
	}
}

/**
  * @brief  Handle PCD IN EP mismatch interrupt.
  * @param  pcd: PCD handle
  * @retval void
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_in_ep_mismatch_interrupt(usbd_pcd_t *pcd, u32 ep_num)
{
	UNUSED(pcd);
	UNUSED(ep_num);
	//unmask InToken received with EP Mismatch
	USB_DEVICE->DIEPMSK &= ~USB_OTG_DIEPMSK_INEPNMM;
}

/**
  * @brief  Handle PCD IN EP NAK effective interrupt.
  * @param  pcd: PCD handle
  * @retval void
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_in_ep_nak_effective_interrupt(usbd_pcd_t *pcd, u32 ep_num)
{
	UNUSED(pcd);
	UNUSED(ep_num);
	USB_DEVICE->DIEPMSK &= ~USB_OTG_DIEPMSK_INEPNEM;
}

/**
  * @brief  Handle PCD IN ISOC incompISOIN interrupt.
  * @param  pcd: PCD handle
  * @retval void
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_incomplete_in_isoc(usbd_pcd_t *pcd)
{
	u32 i ;
	u32 regvalue ;
	usbd_pcd_ep_t *ep;
	for (i = 1U; i < USB_MAX_ENDPOINTS; i++) {
		ep = &(pcd->in_ep[i]);  //in
		if ((ep->type == USB_CH_EP_TYPE_ISOC) && (ep->is_in == 1) && ((USB_INEP(ep->num)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA)) {
			regvalue = USB_INEP(ep->num)->DIEPCTL ;
			if ((regvalue & USB_OTG_DIEPCTL_EONUM_DPID) == 0) {
				regvalue |= USB_OTG_DIEPCTL_SODDFRM;
			} else {
				regvalue |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
			}
			/*
				Fix Isoc In Issue:
				if set bit31=1 again, HW will force to send a ZLP in next in token, and packet lost
				while set 0 not change bit31 value, it is still 1
			*/
			regvalue &= ~(USB_OTG_DIEPCTL_EPENA); //bit31
			USB_INEP(ep->num)->DIEPCTL = regvalue;
		}
	}
}


/**
  * @brief  Handle PCD IN EP interrupt.
  * @param  pcd: PCD handle
  * @retval void
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_in_ep_interrupt(usbd_pcd_t *pcd)
{
	u32 ep_intr;
	u32 epint;
	u32 msk;
	u32 len;
	u32 reg;
	u32 ep_num = 0U;
	usbd_pcd_ep_t *pep;

	/* Read in the device interrupt bits */
	ep_intr = usbd_hal_read_all_in_ep_interrupts(pcd);
	while ((ep_intr != 0U) && (ep_num < USB_MAX_ENDPOINTS)) {
		if ((ep_intr & 0x1U) != 0U) { /* In ITR */
			pep = &pcd->in_ep[ep_num];
			epint = usbd_hal_read_in_ep_interrupts(pcd, (u8)ep_num);
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "IN EP%d 0x%08X\n", ep_num, epint);

			if ((epint & USB_OTG_DIEPINT_XFRC) == USB_OTG_DIEPINT_XFRC) {
				DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "IN EP%d XFRC\n", ep_num);

				msk = 0x1UL << (ep_num & EP_ADDR_MSK);
				USB_DEVICE->DIEPEMPMSK &= ~msk;

				USB_PCD_CLEAR_IN_EP_INTR(ep_num, USB_OTG_DIEPINT_XFRC);

				if (pcd->config.dma_enable) {
					pep->xfer_buff += pep->max_packet_len;
				}

				usb_os_spinunlock(pcd->lock);
				usbd_core_data_in_stage(pcd->dev, (u8)ep_num, pep->xfer_buff, HAL_OK);
				usb_os_spinlock(pcd->lock);

				if (pcd->config.dma_enable) {
					/* this is ZLP, so prepare EP0 for next setup */
					if ((ep_num == 0U) && (pep->xfer_len == 0U)) {
						/* prepare to rx more setup packets */
						(void)usbd_hal_ep0_out_start(pcd);
					}
				}
			}
			if ((epint & USB_OTG_DIEPINT_TOC) == USB_OTG_DIEPINT_TOC) {
				DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "IN EP%d TOC\n", ep_num);
				USB_PCD_CLEAR_IN_EP_INTR(ep_num, USB_OTG_DIEPINT_TOC);
			}
			if ((epint & USB_OTG_DIEPINT_ITTXFE) == USB_OTG_DIEPINT_ITTXFE) {//for share FIFO&Slave Mode
				DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "IN EP%d: ITTXFE[%d]\n", ep_num, pep->data_to_fifo);
				if ((1 == pep->data_to_fifo) && (1 == pep->is_in) &&
					((USB_INEP(pep->num)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA)) {
					DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "IN EP%d: ITTXFE write FIFO\n", ep_num);
					//write data to fifo
					if ((pep->xfer_len > pep->xfer_count) && (pep->xfer_buff != NULL)) {
						len = pep->xfer_len - pep->xfer_count;
						if (len > pep->max_packet_len) {
							len = pep->max_packet_len;
						}
						/*
							while many eps are transmiting, all of them get ITTXFE interrups.
							after writing many packets into the FIFO, maybe there is not enough space
							in the FIFO for the next ep to be written

							if EP has wrote the data to FIFO, set data_to_fifo to zero
							if not, the EP will keep replying NAK to the host, and wait next ITTXFE to write the packet into FIFO
						*/
						reg = USB_GLOBAL->GNPTXSTS;
						if (((reg & USB_OTG_GNPTXSTS_NPTQXSAV_Msk) > 0)
							&& ((reg & USB_OTG_GNPTXSTS_NPTXFSAV_Msk) >= ((len + 3U) / 4U))) {
							usb_hal_write_packet(pep->xfer_buff, pep->num, (u16)len);
							pep->xfer_count += len;
							pep->xfer_buff += len;
							pep->data_to_fifo = 0;
						}
					}
				}
				USB_PCD_CLEAR_IN_EP_INTR(ep_num, USB_OTG_DIEPINT_ITTXFE);
			}
			if ((epint & USB_OTG_DIEPINT_INEPNM) == USB_OTG_DIEPINT_INEPNM) {
				DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "IN EP%d INEPNM\n", ep_num);
				usbd_pcd_handle_in_ep_mismatch_interrupt(pcd, ep_num);
				USB_PCD_CLEAR_IN_EP_INTR(ep_num, USB_OTG_DIEPINT_INEPNM);
			}
			if ((epint & USB_OTG_DIEPINT_INEPNE) == USB_OTG_DIEPINT_INEPNE) {
				DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "IN EP%d INEPNE\n", ep_num);
				usbd_pcd_handle_in_ep_nak_effective_interrupt(pcd, ep_num);
				USB_PCD_CLEAR_IN_EP_INTR(ep_num, USB_OTG_DIEPINT_INEPNE);
			}
			if ((epint & USB_OTG_DIEPINT_EPDISD) == USB_OTG_DIEPINT_EPDISD) {
				DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "IN EP%d EPDISD\n", ep_num);
				usbd_pcd_handle_in_ep_disabled_interrupt(pcd, ep_num);
				USB_PCD_CLEAR_IN_EP_INTR(ep_num, USB_OTG_DIEPINT_EPDISD);
			}
			if ((epint & USB_OTG_DIEPINT_TXFE) == USB_OTG_DIEPINT_TXFE) {
				DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "IN EP%d TXFE\n", ep_num);
				usbd_pcd_handle_in_ep_tx_fifo_empty_interrupt(pcd, ep_num);
			}
		}
		ep_num++;
		ep_intr >>= 1U;
	}
}

/**
  * @brief  Handle PCD RxFIFO non-empty interrupt.
  * @param  pcd: PCD handle
  * @retval void
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_rx_fifo_non_empty_interrupt(usbd_pcd_t *pcd)
{
	usbd_pcd_ep_t *ep;
	u32 reg;
	u32 xfer_cnt;

	USB_PCD_MASK_INTERRUPT(USB_OTG_GINTSTS_RXFLVL);

	reg = USB_GLOBAL->GRXSTSP;
	xfer_cnt = (reg & USB_OTG_GRXSTSP_BCNT) >> USB_OTG_GRXSTSP_BCNT_Pos;

	ep = &pcd->out_ep[reg & USB_OTG_GRXSTSP_EPNUM];

	if (((reg & USB_OTG_GRXSTSP_PKTSTS) >> USB_OTG_GRXSTSP_PKTSTS_Pos) ==  STS_DATA_UPDT) {
		if (xfer_cnt != 0U) {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "RXFLVL EP%d STS_DATA_UPDT %d\n", ep->num, xfer_cnt);
			if ((ep->xfer_buff != NULL) && (ep->xfer_count + xfer_cnt <= ep->xfer_len)) {
				usb_hal_read_packet(ep->xfer_buff, ep->num, (u16)(xfer_cnt & 0xFFFF));
				ep->xfer_buff += xfer_cnt;
				ep->xfer_count += xfer_cnt;
			}
		} else {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "RXFLVL EP%d STS_DATA_UPDT ZLP\n", ep->num);
		}
	} else if (((reg & USB_OTG_GRXSTSP_PKTSTS) >> USB_OTG_GRXSTSP_PKTSTS_Pos) ==  STS_SETUP_UPDT) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "RXFLVL EP%d STS_SETUP_UPDT %d\n", ep->num, xfer_cnt);
		usb_hal_read_packet((u8 *)pcd->setup, ep->num, 8U);
		ep->xfer_count += xfer_cnt;
	} else {
		/* ... */
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "RXFLVL EP%d %d\n", ep->num, xfer_cnt);
	}
	USB_PCD_UNMASK_INTERRUPT(USB_OTG_GINTSTS_RXFLVL);
}

/**
  * @brief  Get IN EP number sequence as per IN Token learn queue
  * @param  pcd: PCD handle
  * @retval void
  */
USB_TEXT_SECTION
static void usbd_pcd_get_in_ep_sequence_from_in_token_queue(usbd_pcd_t *pcd)
{
	UNUSED(pcd);
	u32 i;
	u32 idx;
	u32 cnt;
	int start;
	int end;
	int sort_done;
	u32 dtknqr1;
	u32 dtknqr2;
	u8 intkn_seq[USB_IN_TOKEN_QUEUE_DEPTH];
	u8 seq[USB_MAX_ENDPOINTS];
	u8 temp;
	u32 ndx;

	dtknqr1 = USB_DEVICE->DTKNQR1;
	dtknqr2 = USB_DEVICE->DTKNQR2;

	/* Flush in token learning queue */
	USB_GLOBAL->GRSTCTL |= USB_OTG_GRSTCTL_INTKNQFLSH;

	/* Mark ep senquence[] by 0xff */
	for (i = 0; i < USB_MAX_ENDPOINTS; i++) {
		pcd->in_ep_sequence[i] = 0xFF;
	}

	if ((dtknqr1 & USB_OTG_DTKNQR1_WRAPBIT) != 0) {
		start = (int)(dtknqr1 & USB_OTG_DTKNQR1_INTKNWPTR_Msk);
		end = start - 1;
		if (end < 0) {
			end = USB_IN_TOKEN_QUEUE_DEPTH - 1;
		}
		cnt = USB_IN_TOKEN_QUEUE_DEPTH;
	} else {
		start = 0;
		if ((dtknqr1 & USB_OTG_DTKNQR1_INTKNWPTR_Msk) == 0) {
			// No in tokens received
			return;
		} else {
			end = (int)(dtknqr1 & USB_OTG_DTKNQR1_INTKNWPTR_Msk) - 1;
			cnt = end - start + 1;
		}
	}

	/* Fill seqnum[] by initial values: EP number + 31 */
	for (i = 0; i < USB_MAX_ENDPOINTS; i++) {
		seq[i] = i + 31;
	}
	/* Fill intkn_seq[] from in_tkn_epnums[0] */
	for (i = 0; i < 6; i++) {
		intkn_seq[i] = (dtknqr1 >> (8 + i * 4)) & 0xF;
	}

	for (i = 6; i < USB_IN_TOKEN_QUEUE_DEPTH; i++) {
		intkn_seq[i] = (dtknqr2 >> ((i - 6) * 4)) & 0xF;
	}

#if IN_TOKEN_PREDICT_DEBUG_EN
	for (i = 0; i < USB_IN_TOKEN_QUEUE_DEPTH; i++) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "Raw INTKN[%d]=EP%d\n", i, intkn_seq[i]);
	}

	DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "Start=%d end=%d\n", start, end);
#endif

	/* Update seqnum based on intkn_seq[] */
	i = 0;
	idx = start;
	do {
		seq[intkn_seq[idx]] = i;
		idx++;
		i++;
		if (idx == USB_IN_TOKEN_QUEUE_DEPTH) {
			idx = 0;
		}
	} while (i < cnt);

#if IN_TOKEN_PREDICT_DEBUG_EN
	for (i = 0; i < USB_MAX_ENDPOINTS; i++) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "Unsorted SEQ[%d]=%d\n", i, seq[i]);
	}
#endif

	/* Sort seqnum[] */
	sort_done = 0;
	while (!sort_done) {
		sort_done = 1;
		for (i = 0U; i < USB_MAX_ENDPOINTS - 1U; i++) {
			if (seq[i] > seq[i + 1]) {
				temp = seq[i];
				seq[i] = seq[i + 1];
				seq[i + 1] = temp;
				sort_done = 0;
			}
		}
	}

	ndx = start + seq[0];
	if (ndx >= USB_IN_TOKEN_QUEUE_DEPTH) {
		ndx = ndx % USB_IN_TOKEN_QUEUE_DEPTH;
	}
	pcd->first_in_nextep_seq = intkn_seq[ndx];

#if IN_TOKEN_PREDICT_DEBUG_EN
	for (i = 0; i < USB_MAX_ENDPOINTS; i++) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "Sorted SEQ[%d]=%d\n", i, seq[i]);
	}
#endif

	/* Update nextep_seq[] by EP numbers */
	for (i = 0; i < USB_MAX_ENDPOINTS; i++) {
		idx = start + i;
		if (seq[i] < 31) {
			idx = start + seq[i];
			if (idx >= USB_IN_TOKEN_QUEUE_DEPTH) {
				idx = idx % USB_IN_TOKEN_QUEUE_DEPTH;
			}
			pcd->in_ep_sequence[i] = intkn_seq[idx];
		}
	}

#if IN_TOKEN_PREDICT_DEBUG_EN
	for (i = 0; i < USB_MAX_ENDPOINTS; i++) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "INTKN[%d]=%d\n", i, pcd->in_ep_sequence[i]);
	}
#endif
}

/**
  * @brief  Handle PCD EP non-periodical TxFIFO empty interrupt.
  * @param  pcd: PCD handle
  * @param  ep_num: Endpoint number
  * @retval void
  */
USB_TEXT_SECTION
static u8 usbd_pcd_handle_ep_np_tx_fifo_empty_interrupt(usbd_pcd_t *pcd, u8 ep_num)
{
	usbd_pcd_ep_t *ep;
	u32 len;
	u32 reg;
	u8 status = 0;

	ep = &pcd->in_ep[ep_num];

	if ((ep->xfer_len == 0) && (ep->is_zlp == 0)) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "NPTxFEmp: token received for EP%d when TxFIFO is not ready\n", ep_num);
		return 0;
	}

	if ((ep->xfer_count > ep->xfer_len) || ((ep->xfer_len > 0) && (ep->xfer_buff == NULL))) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "NPTxFEmp: invalid parameter\n");
		return 0;
	}

	if ((ep->xfer_len == 0) || (ep->xfer_count == ep->xfer_len)) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "NPTxFEmp: EP%d false alarm\n", ep_num);
		return 0;
	}

	len = ep->xfer_len - ep->xfer_count;
	if (len > ep->max_packet_len) {
		len = ep->max_packet_len;
	}

	DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "NPTxFEmp EP%d xfer_len=%d xfer_count=%d len=%d\n", ep_num, ep->xfer_len, ep->xfer_count, len);

	/* While there is space in the queue and space in the FIFO and
	 * More data to tranfer, Write packets to the Tx FIFO */
	reg = USB_GLOBAL->GNPTXSTS;

	while (((reg & USB_OTG_GNPTXSTS_NPTQXSAV_Msk) > 0)
		   && ((reg & USB_OTG_GNPTXSTS_NPTXFSAV_Msk) >= ((len + 3U) / 4U))
		   && (ep->xfer_count < ep->xfer_len)) {
		if (!pcd->config.dma_enable) {
			usb_hal_write_packet(ep->xfer_buff, ep->num, (u16)len);
		}
		ep->xfer_count += len;
		ep->xfer_buff += len;
		len = ep->xfer_len - ep->xfer_count;
		if (len > ep->max_packet_len) {
			len = ep->max_packet_len;
		}
		reg = USB_GLOBAL->GNPTXSTS;
	}

	if (ep->xfer_len > ep->xfer_count) {
		status = 1;
	}

	return status;
}

/**
  * @brief  Handle PCD non-periodical TxFIFO empty interrupt.
  * @param  pcd: PCD handle
  * @retval void
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_np_tx_fifo_empty_interrupt(usbd_pcd_t *pcd)
{
	u8 ep_num;
	u32 i;
	u8 status;
#if IN_TOKEN_PREDICT_WA_EN
	usbd_pcd_ep_t *ep;
#endif

	USB_GLOBAL->GINTMSK &= ~USB_OTG_GINTMSK_NPTXFEM;

	usbd_pcd_get_in_ep_sequence_from_in_token_queue(pcd);

	/* TX as per the prediction result of in token learn queue. */
	for (i = 0; i < USB_MAX_ENDPOINTS; i++) {
		ep_num = pcd->in_ep_sequence[i];
		if (ep_num != 0xFF) {
			if (ep_num < USB_MAX_ENDPOINTS) {
				status = usbd_pcd_handle_ep_np_tx_fifo_empty_interrupt(pcd, ep_num);
				if (status) {
					USB_GLOBAL->GINTMSK |= USB_OTG_GINTMSK_NPTXFEM;
				}
			} else {
				DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "Invalid EP num %d in IN token learn queue\n", ep_num);
			}
		}
	}

#if IN_TOKEN_PREDICT_WA_EN
	/* Workaround: Sometimes in tokens cannot be caught by SW ISR after NPTXFE interrupt especially when multiple NP IN endpoints are enabled.
	   However, this will result in infinite interrupts if usbd_ep_transmit/usbd_ep0_transmit improperly called without in tokens on the bus. */
	if ((USB_GLOBAL->GINTMSK & USB_OTG_GINTMSK_NPTXFEM) == 0) {
		for (i = 0; i < USB_MAX_ENDPOINTS; i++) {
			ep = &pcd->in_ep[i];
			if ((ep->tx_fifo_num == 0) && (ep->xfer_len > ep->xfer_count) && ((USB_INEP(i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) != 0)) {
#ifdef USBD_XFER_ERROR_DETECT_EN
				if (ep->nptx_err_cnt < pcd->config.nptx_max_err_cnt[i]) {
					if (++ep->nptx_err_cnt == pcd->config.nptx_max_err_cnt[i]) {
						DBG_PRINTF(MODULE_USB_OTG, LEVEL_WARN, "EP%d TX timeout %d: xfer_len=%d, xfer_count=%d\n", i, ep->nptx_err_cnt, ep->xfer_len, ep->xfer_count);
						usb_os_spinunlock(pcd->lock);
						usbd_core_data_in_stage(pcd->dev, (u8)i, ep->xfer_buff, HAL_TIMEOUT);
						usb_os_spinlock(pcd->lock);
					} else {
						USB_GLOBAL->GINTMSK |= USB_OTG_GINTMSK_NPTXFEM;
						DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "EP%d TXE miss %d: xfer_len=%d, xfer_count=%d\n", i, ep->nptx_err_cnt, ep->xfer_len, ep->xfer_count);
					}
				}
#else
				USB_GLOBAL->GINTMSK |= USB_OTG_GINTMSK_NPTXFEM;
				DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "EP%d TXE miss: xfer_len=%d, xfer_count=%d\n", i, ep->xfer_len, ep->xfer_count);
#endif
				break;
			}
		}
	}
#endif
}

/**
  * @brief  Handle PCD EP mismatch interrupt.
  * @param  pcd: PCD handle
  * @retval void
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_ep_mismatch_interrupt(usbd_pcd_t *pcd)
{
	u32 reg;
	u32 gintsts = usb_hal_read_interrupts();

	if (!(gintsts & (USB_OTG_GINTSTS_GINAKEFF)) && (++pcd->nptx_epmis_cnt > pcd->config.nptx_max_epmis_cnt)) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "epmis count=%d\n", pcd->config.nptx_max_epmis_cnt);

		pcd->nptx_epmis_cnt = 0;
		pcd->start_predict = 1;

		/* Disable EP Mismatch interrupt */
		/* Enable the Global IN NAK Effective Interrupt */
		reg = USB_GLOBAL->GINTMSK;
		reg &= ~(USB_OTG_GINTMSK_EPMISM);
		reg |= USB_OTG_GINTMSK_GINAKEFFM;
		USB_GLOBAL->GINTMSK = reg;

		/* Set the global non-periodic IN NAK handshake */
		USB_DEVICE->DCTL |= USB_OTG_DCTL_SGINAK;
	}
}
/**
  * @brief  Handle PCD EP in nak effective interrupt.
  * @param  pcd: PCD handle
  * @retval void
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_in_nak_effective(usbd_pcd_t *pcd)
{
	usbd_pcd_ep_t *ep;
	u32 reg;
	u8 i;

	/* Disable all active IN EPs(use non-periodic shared TxFifo) */
	for (i = 0; i < USB_MAX_ENDPOINTS; i++) {
		ep = &(pcd->in_ep[i]);
		reg = USB_INEP(ep->num)->DIEPCTL;

		if ((!(ep->is_ptx)) && ((reg & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA)) {
			//count for how many np eps
			if (pcd->start_predict > 0) {
				pcd->start_predict++;
			}
			reg |= (USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK);
			USB_INEP(ep->num)->DIEPCTL = reg;
		}
	}

	/* Disable the Global IN NAK Effective Interrupt */
	USB_GLOBAL->GINTMSK &= ~(USB_OTG_GINTMSK_GINAKEFFM);
}

/**
  * @brief  Handle PCD wakeup interrupt.
  * @param  pcd: PCD handle
  * @retval void
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_wakeup_interrupt(usbd_pcd_t *pcd)
{
	/* Clear the Remote Wake-up Signaling */
	USB_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;

	if (pcd->lpm_state == LPM_L1) {
		pcd->lpm_state = LPM_L0;
		// FIXME: Do nothing
	} else {
		usb_os_spinunlock(pcd->lock);
		usbd_core_resume(pcd->dev);
		usb_os_spinlock(pcd->lock);
	}
}

/**
  * @brief  Handle SOF interrupt
  * @param  pcd: PCD handle
  * @retval None
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_sof_interrupt(usbd_pcd_t *pcd)
{
	usb_os_spinunlock(pcd->lock);
	usbd_core_sof(pcd->dev);
	usb_os_spinlock(pcd->lock);
}

/**
  * @brief  Handle EOPF interrupt
  * @param  pcd: PCD handle
  * @retval None
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_eopf_interrupt(usbd_pcd_t *pcd)
{
	usbd_core_eopf(pcd->dev);
}

/**
  * @brief  Handle SRQ interrupt
  * @param  pcd: PCD handle
  * @retval None
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_srq_interrupt(usbd_pcd_t *pcd)
{
	usb_os_spinunlock(pcd->lock);
	usbd_core_connected(pcd->dev);
	usb_os_spinlock(pcd->lock);
}

/**
  * @brief  Handle OTG interrupt
  * @param  pcd: PCD handle
  * @retval None
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_otg_interrupt(usbd_pcd_t *pcd)
{
	u32 reg = USB_GLOBAL->GOTGINT;

	if ((reg & USB_OTG_GOTGINT_SEDET) == USB_OTG_GOTGINT_SEDET) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "GOTGINT_SEDET\n");
		usb_os_spinunlock(pcd->lock);
		usbd_core_disconnected(pcd->dev);
		usb_os_spinlock(pcd->lock);
	}

	USB_GLOBAL->GOTGINT |= reg;
}

/**
  * @brief  Handles PCD interrupt request.
  * @param  pcd: PCD handle
  * @retval HAL status
  */
USB_TEXT_SECTION
static void usbd_pcd_handle_interrupt(usbd_pcd_t *pcd)
{
	/* ensure that we are in device mode */
	if (usb_hal_get_otg_mode() == USB_OTG_MODE_DEVICE) {

		usb_os_spinlock(pcd->lock);

		u32  gintsts =  usb_hal_read_interrupts();

		/* avoid spurious interrupt */
		if (gintsts == 0U) {
			usb_os_spinunlock(pcd->lock);
			return;
		}

		//DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "====== IRQ 0x%08X =======\n", usb_hal_read_interrupts());

		if (gintsts & (USB_OTG_GINTSTS_IEPINT)) {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "IEPINT\n");
			usbd_pcd_handle_in_ep_interrupt(pcd);
		}

		if (gintsts & (USB_OTG_GINTSTS_OEPINT)) {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "OEPINT\n");
			usbd_pcd_handle_out_ep_interrupt(pcd);
		}

		if (gintsts & (USB_OTG_GINTSTS_IISOIXFR)) {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "incompISOIN\n");
			usbd_pcd_handle_incomplete_in_isoc(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_IISOIXFR);
		}

		/* Handle Resume Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_WKUINT)) {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "WKUINT\n");
			usbd_pcd_handle_wakeup_interrupt(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_WKUINT);
		}

		/* Handle Suspend Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_USBSUSP)) {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "USBSUSP\n");
			usbd_pcd_handle_suspend_interrupt(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_USBSUSP);
		}
		/* Handle Reset Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_USBRST)) {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "USBRST\n");
			usbd_pcd_handle_reset_interrupt(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_USBRST);
		}

		/* Handle Enumeration done Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_ENUMDNE)) {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "ENUMDNE\n");
			usbd_pcd_handle_enum_done_interrupt(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_ENUMDNE);
		}

		/* Handle RxQLevel Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_RXFLVL)) {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "RXFLVL\n");
			usbd_pcd_handle_rx_fifo_non_empty_interrupt(pcd);
		}

		/* Handle NP TxFIFO Empty Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_NPTXFE)) {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "NPTXFE\n");
			usbd_pcd_handle_np_tx_fifo_empty_interrupt(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_NPTXFE);
		}

		/* Handle EPMIS Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_EPMIS)) {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "EPMIS\n");
			usbd_pcd_handle_ep_mismatch_interrupt(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_EPMIS);
		}

		/* Handle GINAKEFF Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_GINAKEFF)) {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "GINAKEFF\n");
			usbd_pcd_handle_in_nak_effective(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_GINAKEFF);
		}

		/* Handle SOF Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_SOF)) {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "SOF\n");
			usbd_pcd_handle_sof_interrupt(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_SOF);
		}

		/* Handle EOPF Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_EOPF)) {
			//DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "EOPF\n");
			usbd_pcd_handle_eopf_interrupt(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_EOPF);
		}

		/* Handle Connection event Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_SRQINT)) {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "SRQINT\n");
			usbd_pcd_handle_srq_interrupt(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_SRQINT);
		}

		/* Handle Disconnection event Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_OTGINT)) {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "OTGINT\n");
			usbd_pcd_handle_otg_interrupt(pcd);
		}

		usb_os_spinunlock(pcd->lock);
	}
}


/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize PCD
  * @param  dev: USB device instance
  * @param  config: USB device configuration
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_pcd_init(usb_dev_t *dev, usbd_config_t *config)
{
	u8 i;
	u8 ret = HAL_OK;
	usbd_pcd_t *pcd = &usbd_pcd;

	dev->pcd = &usbd_pcd;
	pcd->dev = dev;

	rtw_memcpy((void *)&pcd->config, (void *)config, sizeof(usbd_config_t));

	for (i = 0U; i < USB_MAX_ENDPOINTS; i++) {
		if (pcd->config.nptx_max_err_cnt[i] == 0U) {
			pcd->config.nptx_max_err_cnt[i] = USBD_NPTX_DEF_ERR_CNT;
		}
	}

	if (pcd->pcd_state == HAL_PCD_STATE_RESET) {
		/* Allocate lock resource and initialize it */
		pcd->lock = usb_os_spinlock_alloc();
		if (pcd->lock == NULL) {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "PCD lock init fail\n");
			pcd->pcd_state = HAL_PCD_STATE_ERROR;
			return HAL_ERR_MEM;
		}

		/* Init the low level hardware: GPIO, CLOCK, PHY... */
		ret = usb_chip_init();
		if (ret != HAL_OK) {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "USB low-level init fail\n");
			pcd->pcd_state = HAL_PCD_STATE_ERROR;
			return ret;
		}
	}

	pcd->pcd_state = HAL_PCD_STATE_BUSY;
	pcd->phy_initialized = 0;
	pcd->isr_initialized = 0;
	pcd->setup = (u8 *)rtw_zmalloc(USBD_SETUP_PACKET_BUF_LEN);
	if (pcd->setup == NULL) {
		return HAL_ERR_MEM;
	}
	if (config->dma_enable && (!USB_IS_MEM_DMA_ALIGNED(pcd->setup))) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "Setup buffer is not 32 bytes aligned in dma mode!\n");
		return HAL_ERR_MEM;
	}

	/* Disable the Interrupts */
	usb_hal_disable_global_interrupt();

	/* Init the Core (common init.) */
	ret = usb_hal_core_init(config->dma_enable);
	if (ret != HAL_OK) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "Core init fail\n");
		pcd->pcd_state = HAL_PCD_STATE_ERROR;
		return ret;
	}

	/* Force Device Mode*/
	usb_hal_set_otg_mode((usb_otg_mode_t)USB_OTG_MODE_DEVICE);

	/* Init Device */
	ret = usbd_hal_device_init(pcd);
	if (ret != HAL_OK) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "Device init fail\n");
		pcd->pcd_state = HAL_PCD_STATE_ERROR;
		return ret;
	}

	/* Init endpoints structures */
	for (i = 0U; i < USB_MAX_ENDPOINTS; i++) {
		/* Init ep structure */
		pcd->in_ep[i].is_in = 1U;
		pcd->in_ep[i].num = i;
		if (pcd->ded_tx_fifo_en) {
			pcd->in_ep[i].tx_fifo_num = i;
		} else {
			pcd->in_ep[i].tx_fifo_num = 0U;
		}
		/* Control until ep is activated */
		pcd->in_ep[i].type = USB_CH_EP_TYPE_CTRL;
		pcd->in_ep[i].max_packet_len = 0U;
		pcd->in_ep[i].xfer_buff = 0U;
		pcd->in_ep[i].xfer_len = 0U;
		pcd->in_ep[i].is_zlp = 0U;
		pcd->in_ep[i].is_ptx = 0U;
		pcd->in_ep[i].is_initialized = 0U;
	}

	for (i = 0U; i < USB_MAX_ENDPOINTS; i++) {
		pcd->out_ep[i].is_in = 0U;
		pcd->out_ep[i].num = i;
		/* Control until ep is activated */
		pcd->out_ep[i].type = USB_CH_EP_TYPE_CTRL;
		pcd->out_ep[i].max_packet_len = 0U;
		pcd->out_ep[i].xfer_buff = 0U;
		pcd->out_ep[i].xfer_len = 0U;
		pcd->out_ep[i].is_zlp = 0U;
		pcd->out_ep[i].is_initialized = 0U;
	}

	pcd->address = 0U;
	pcd->pcd_state = HAL_PCD_STATE_READY;
	(void)usbd_hal_disconnect(pcd);

#if CONFIG_USB_PHY
#if CONFIG_USB_OTG
	ret = usb_hal_enable_otg();
	if (ret != HAL_OK) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "OTG enable fail\n");
		pcd->pcd_state = HAL_PCD_STATE_ERROR;
		return ret;
	}
#endif

	ret = usb_hal_calibrate(USB_OTG_MODE_DEVICE);
	if (ret != HAL_OK) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "PHY calibration fail\n");
		pcd->pcd_state = HAL_PCD_STATE_ERROR;
		return ret;
	}
#endif

#if defined (CONFIG_RTL8720F)
	DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "Setup TxFIFO\n");
	//usbd_hal_set_tx_fifo(pcd, 0, 32);
	usbd_hal_set_tx_fifo(pcd, 1, 16);
	usbd_hal_set_tx_fifo(pcd, 2, 256);
	usbd_hal_set_tx_fifo(pcd, 3, 32);
	usbd_hal_set_tx_fifo(pcd, 4, 256);
#elif (defined(CONFIG_AMEBAD2) || defined(CONFIG_RTL8721D))
	usbd_hal_set_rx_fifo(pcd, 504);
	usbd_hal_set_tx_fifo(pcd, 0, 256);
	usbd_hal_set_tx_fifo(pcd, 1, 256);
#endif

	ret = usbd_pcd_interrupt_init(pcd);
	if (ret != HAL_OK) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "Interrupt init fail\n");
		pcd->pcd_state = HAL_PCD_STATE_ERROR;
		return ret;
	}

	return HAL_OK;
}

/**
  * @brief  DeInitialize PCD
  * @param  pcd: PCD handle
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_pcd_deinit(usb_dev_t *dev)
{
	u8 result = HAL_OK;

	usbd_pcd_t *pcd = dev->pcd;

	if (pcd == NULL) {
		return HAL_OK;
	}

	usbd_pcd_interrupt_deinit(pcd);

	pcd->pcd_state = HAL_PCD_STATE_BUSY;

	/* Stop Device */
	result = usbd_pcd_stop(pcd);

	usb_chip_deinit();

	usb_os_spinlock_free(pcd->lock);

	if (pcd->setup != NULL) {
		rtw_free(pcd->setup);
		pcd->setup = NULL;
	}

	pcd->lock = NULL;
	dev->pcd = NULL;

	pcd->pcd_state = HAL_PCD_STATE_RESET;

	return result;
}

/**
  * @brief  Start the PCD
  * @param  pcd: PCD handle
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_pcd_start(usbd_pcd_t *pcd)
{
	usb_os_spinlock(pcd->lock);
	(void)usbd_hal_connect(pcd);
	usb_hal_enable_global_interrupt();
	usb_os_spinunlock(pcd->lock);
	return HAL_OK;
}

/**
  * @brief  Stop the PCD
  * @param  pcd: PCD handle
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_pcd_stop(usbd_pcd_t *pcd)
{
	usb_os_spinlock(pcd->lock);

	if (pcd->pcd_state != HAL_PCD_STATE_STOP) {
		usb_hal_disable_global_interrupt();
		if (usbd_hal_device_stop(pcd) != HAL_OK) {
			usb_os_spinunlock(pcd->lock);
			return HAL_ERR_HW;
		}
		(void)usbd_hal_disconnect(pcd);
		pcd->pcd_state = HAL_PCD_STATE_STOP;
	}

	usb_os_spinunlock(pcd->lock);

	return HAL_OK;
}

/**
  * @brief  Connect the USB device
  * @param  pcd: PCD handle
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_pcd_dev_connected(usbd_pcd_t *pcd)
{
	usb_os_spinlock(pcd->lock);
	(void)usbd_hal_connect(pcd);
	usb_os_spinunlock(pcd->lock);
	return HAL_OK;
}

/**
  * @brief  Disconnect the USB device.
  * @param  pcd: PCD handle
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_pcd_dev_desconnected(usbd_pcd_t *pcd)
{
	usb_os_spinlock(pcd->lock);
	(void)usbd_hal_disconnect(pcd);
	usb_os_spinunlock(pcd->lock);
	return HAL_OK;
}

/**
  * @brief  Set the USB Device address.
  * @param  pcd: PCD handle
  * @param  address: New device address
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_pcd_set_address(usbd_pcd_t *pcd, u8 address)
{
	usb_os_spinlock(pcd->lock);
	pcd->address = address;
	(void)usbd_hal_set_device_address(pcd, address);
	usb_os_spinunlock(pcd->lock);
	return HAL_OK;
}

/**
  * @brief  Open and configure an endpoint.
  * @param  pcd: PCD handle
  * @param  ep_addr: Endpoint address
  * @param  ep_mps: Endpoint max packet size
  * @param  ep_type: Endpoint type
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_pcd_ep_init(usbd_pcd_t *pcd, u8 ep_addr, u16 ep_mps, u8 ep_type)
{
	u8 is_ptx = 0;
	u8 ret = HAL_OK;
	usbd_pcd_ep_t *ep;

	if ((ep_addr & 0x80U) == 0x80U) {
		ep = &pcd->in_ep[ep_addr & EP_ADDR_MSK];
		ep->is_in = 1U;
		is_ptx = (ep_type == USB_CH_EP_TYPE_ISOC) ||
				 ((ep_type == USB_CH_EP_TYPE_INTR) && (pcd->config.intr_use_ptx_fifo != 0U) && (pcd->ded_tx_fifo_en == 0U));
	} else {
		ep = &pcd->out_ep[ep_addr & EP_ADDR_MSK];
		ep->is_in = 0U;
	}

	ep->num = ep_addr & EP_ADDR_MSK;
	ep->max_packet_len = ep_mps;
	ep->type = ep_type;
	ep->is_initialized = 1U;
	ep->is_ptx = is_ptx;
	if (ep->is_in != 0U) {
		if (pcd->ded_tx_fifo_en) {
			ep->tx_fifo_num = ep->num;
		} else {
			if ((ep_type == USB_CH_EP_TYPE_ISOC) ||
				((ep_type == USB_CH_EP_TYPE_INTR) && (pcd->config.intr_use_ptx_fifo != 0U))) {
				ep->tx_fifo_num = 1;
			} else {
				ep->tx_fifo_num = 0;
			}
		}
	}

	/* Set initial data PID. */
	if (ep_type == USB_CH_EP_TYPE_BULK) {
		ep->data_pid_start = 0U;
	}

	usb_os_spinlock(pcd->lock);
	(void)usbd_hal_ep_activate(pcd, ep);
	usb_os_spinunlock(pcd->lock);

	return ret;
}

/**
  * @brief  Deactivate an endpoint.
  * @param  pcd: PCD handle
  * @param  ep_addr: Endpoint address
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_pcd_ep_deinit(usbd_pcd_t *pcd, u8 ep_addr)
{
	usbd_pcd_ep_t *ep;

	if ((ep_addr & 0x80U) == 0x80U) {
		ep = &pcd->in_ep[ep_addr & EP_ADDR_MSK];
		ep->is_in = 1U;
	} else {
		ep = &pcd->out_ep[ep_addr & EP_ADDR_MSK];
		ep->is_in = 0U;
	}
	ep->num = ep_addr & EP_ADDR_MSK;
	ep->is_initialized = 0U;
	ep->is_ptx = 0U;

	usb_os_spinlock(pcd->lock);
	(void)usbd_hal_ep_deactivate(pcd, ep);
	usb_os_spinunlock(pcd->lock);
	return HAL_OK;
}

/**
  * @brief  Receive an amount of data.
  * @param  pcd: PCD handle
  * @param  ep_addr: Endpoint address
  * @param  buf: Pointer to the reception buffer
  * @param  len: Amount of data to be received
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_pcd_ep_receive(usbd_pcd_t *pcd, u8 ep_addr, u8 *buf, u32 len)
{
	usbd_pcd_ep_t *ep;
	u32 dma = pcd->config.dma_enable;
	u8 ep_num = USB_EP_NUM(ep_addr);

	if ((dma != 0) && (buf != NULL) && (len != 0)) {
		if (USB_IS_MEM_DMA_ALIGNED(buf)) {
			DCache_Clean((u32)buf, len);
		} else {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "Receive buffer is not 32 bytes aligned in dma mode!\n");
			return HAL_ERR_MEM;
		}
	}

	ep = &pcd->out_ep[ep_addr & EP_ADDR_MSK];

	/*setup and start the Xfer */
	ep->xfer_buff = buf;
	ep->xfer_len = len;
	ep->xfer_count = 0U;
	ep->is_in = 0U;
	ep->is_zlp = (len == 0) ? 1 : 0;
	ep->num = ep_addr & EP_ADDR_MSK;

	if (dma) {
		ep->dma_addr = (u32)buf;
	}

	usb_os_spinlock_safe(pcd->lock);

	if ((ep_addr & EP_ADDR_MSK) == 0U) {
		(void)usbd_hal_ep0_start_transfer(pcd, ep);
	} else {
		(void)usbd_hal_ep_start_transfer(pcd, ep);
	}

	usb_os_spinunlock_safe(pcd->lock);

	return HAL_OK;
}

/**
  * @brief  Get Received Data Size
  * @param  pcd: PCD handle
  * @param  ep_addr: Endpoint address
  * @retval Data Size
  */
USB_TEXT_SECTION
u32 usbd_pcd_ep_get_rx_data_size(usbd_pcd_t *pcd, u8 ep_addr)
{
	return pcd->out_ep[ep_addr & EP_ADDR_MSK].xfer_count;
}

/**
  * @brief  Send an amount of data
  * @param  pcd: PCD handle
  * @param  ep_addr: Endpoint address
  * @param  buf: Pointer to the transmission buffer
  * @param  len: Amount of data to be sent
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_pcd_ep_transmit(usbd_pcd_t *pcd, u8 ep_addr, u8 *buf, u32 len)
{
	usbd_pcd_ep_t *ep;
	u32 dma = pcd->config.dma_enable;
	u8 ep_num = USB_EP_NUM(ep_addr);

	if ((dma != 0) && (buf != NULL) && (len != 0)) {
		if (USB_IS_MEM_DMA_ALIGNED(buf)) {
			DCache_Clean((u32)buf, len);
		} else {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "Transmit buffer is not 32 bytes aligned in dma mode!\n");
			return HAL_ERR_MEM;
		}
	}

	ep = &pcd->in_ep[ep_addr & EP_ADDR_MSK];

	/* Setup and start the transfer */
	ep->xfer_buff = buf;
	ep->xfer_len = len;
	ep->xfer_count = 0U;
	ep->is_in = 1U;
	ep->num = ep_addr & EP_ADDR_MSK;
	ep->is_zlp = (len == 0) ? 1 : 0;

	if (dma) {
		ep->dma_addr = (u32)buf;
	}

#ifdef USBD_XFER_ERROR_DETECT_EN
	ep->nptx_err_cnt = 0;
#endif

	usb_os_spinlock_safe(pcd->lock);

	if ((ep_addr & EP_ADDR_MSK) == 0U) {
		(void)usbd_hal_ep0_start_transfer(pcd, ep);
	} else {
		(void)usbd_hal_ep_start_transfer(pcd, ep);
	}

	usb_os_spinunlock_safe(pcd->lock);

	return HAL_OK;
}

/**
  * @brief  Set a STALL condition over an endpoint
  * @param  pcd: PCD handle
  * @param  ep_addr: Endpoint address
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_pcd_ep_set_stall(usbd_pcd_t *pcd, u8 ep_addr)
{
	usbd_pcd_ep_t *ep;

	if (((u32)ep_addr & EP_ADDR_MSK) > USB_MAX_ENDPOINTS) {
		return HAL_ERR_PARA;
	}

	if ((0x80U & ep_addr) == 0x80U) {
		ep = &pcd->in_ep[ep_addr & EP_ADDR_MSK];
		ep->is_in = 1U;
	} else {
		ep = &pcd->out_ep[ep_addr];
		ep->is_in = 0U;
	}

	ep->is_stall = 1U;
	ep->num = ep_addr & EP_ADDR_MSK;

	usb_os_spinlock_safe(pcd->lock);

	(void)usbd_hal_ep_set_stall(pcd, ep);
	if ((ep_addr & EP_ADDR_MSK) == 0U) {
		(void)usbd_hal_ep0_out_start(pcd);
	}

	usb_os_spinunlock_safe(pcd->lock);

	return HAL_OK;
}

/**
  * @brief  Clear a STALL condition over in an endpoint
  * @param  pcd: PCD handle
  * @param  ep_addr: Endpoint address
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_pcd_ep_clear_stall(usbd_pcd_t *pcd, u8 ep_addr)
{
	usbd_pcd_ep_t *ep;

	if (((u32)ep_addr & 0x0FU) > USB_MAX_ENDPOINTS) {
		return HAL_ERR_PARA;
	}

	if ((0x80U & ep_addr) == 0x80U) {
		ep = &pcd->in_ep[ep_addr & EP_ADDR_MSK];
		ep->is_in = 1U;
	} else {
		ep = &pcd->out_ep[ep_addr & EP_ADDR_MSK];
		ep->is_in = 0U;
	}

	ep->is_stall = 0U;
	ep->num = ep_addr & EP_ADDR_MSK;

	usb_os_spinlock_safe(pcd->lock);
	(void)usbd_hal_ep_clear_stall(pcd, ep);
	usb_os_spinunlock_safe(pcd->lock);

	return HAL_OK;
}

/**
  * @brief  Flush an endpoint
  * @param  pcd: PCD handle
  * @param  ep_addr: Endpoint address
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_pcd_ep_flush(usbd_pcd_t *pcd, u8 ep_addr)
{
	usb_os_spinlock(pcd->lock);

	if ((ep_addr & 0x80U) == 0x80U) {
		usb_hal_flush_tx_fifo((u32)ep_addr & EP_ADDR_MSK);
	} else {
		usb_hal_flush_rx_fifo();
	}

	usb_os_spinunlock(pcd->lock);

	return HAL_OK;
}

