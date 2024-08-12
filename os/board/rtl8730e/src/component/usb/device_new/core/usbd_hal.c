/**
  ******************************************************************************
  * @file    usbd_hal.c
  * @author  Realsil WLAN5 Team
  * @brief   This file provides the functionalities of USBD HAL layer
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
#include "usbd_hal.h"

/* Private defines -----------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Set the USB turnaround time
  * @param  pcd: PCD Instance
  * @retval USB turnaround time In PHY Clocks number
  */
USB_TEXT_SECTION
u8 usbd_hal_set_turnaround_time(usbd_pcd_t *pcd)
{
	u32 UsbTrd;

	/* The USBTRD is configured according to the tables below, depending on AHB frequency
	used by application. In the low AHB frequency range it is used to stretch enough the USB response
	time to IN tokens, the USB turnaround time, so to compensate for the longer AHB read access
	latency to the Data FIFO */
	if (pcd->config.speed == USB_SPEED_HIGH) {
		UsbTrd = USBD_HS_TRDT_VALUE;
	} else {
		UsbTrd = USBD_FS_TRDT_VALUE;
	}

	USB_GLOBAL->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;
	USB_GLOBAL->GUSBCFG |= (u32)((UsbTrd << 10) & USB_OTG_GUSBCFG_TRDT);

	return HAL_OK;
}

/**
  * @brief  Predict next ep for DMA mode
  * @param  pcd: PCD Instance
  * @param  ep: pointer to endpoint structure
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_hal_predict_next_ep(usbd_pcd_t *pcd, usbd_pcd_ep_t *ep)
{
	u32 reg;
	u32 ep_num = (u32)ep->num;

	reg = USB_INEP(ep_num)->DIEPCTL;
	reg &= ~USB_OTG_DIEPCTL_NEXTEP_Msk;
	reg |= pcd->nextep_seq[ep_num] << USB_OTG_DIEPCTL_NEXTEP_Pos;
	USB_INEP(ep_num)->DIEPCTL = reg;

	return HAL_OK;
}

/**
  * @brief  Reset IN Token Learn Queue
  * @param  pcd: PCD Instance
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_hal_reset_in_token_queue(usbd_pcd_t *pcd)
{
	u32 i;
	u32 reg;

	pcd->in_ep_sequence[0] = 0;
	for (i = 1U; i < USB_MAX_ENDPOINTS; i++) {
		pcd->in_ep_sequence[i] = 0xFF;
	}

	if (pcd->config.dma_enable == 1U) {
		pcd->start_predict = 0;
		pcd->first_in_nextep_seq = 0;
		pcd->nextep_seq[0] = 0;
		for (i = 1U; i < USB_MAX_ENDPOINTS; i++) {
			pcd->nextep_seq[i] = 0xFF;
		}
	}

	USB_GLOBAL->GRSTCTL |= USB_OTG_GRSTCTL_INTKNQFLSH;

	// FIXME: init nextep data
	reg = USB_INEP(0)->DIEPCTL;
	reg &= ~USB_OTG_DIEPCTL_NEXTEP_Msk;
	reg |= 0 << USB_OTG_DIEPCTL_NEXTEP_Pos;
	USB_INEP(0)->DIEPCTL = reg;

	reg = USB_DEVICE->DCFG;
	reg &= ~USB_OTG_DCFG_EPMISCNT_Msk;
	reg |= (USBD_EPMIS_CNT << USB_OTG_DCFG_EPMISCNT_Pos);  // two IN EP, CTRL IN + BULK IN + 1
	USB_DEVICE->DCFG = reg;

	return HAL_OK;
}

/**
  * @brief  Initializes the USB_OTG controller registers for device mode
  * @param  pcd: PCD Instance
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_hal_device_init(usbd_pcd_t *pcd)
{
	u8 ret = HAL_OK;
	usbd_config_t *cfg = &pcd->config;
	u32 i;
	u32 reg;

	pcd->ded_tx_fifo_en = ((USB_GLOBAL->GHWCFG4 & USB_OTG_GHWCFG4_DEDFIFO) == 0) ? 0 : 1;

	/* Restart the Phy Clock */
	USB_PCGCCTL &= ~USB_OTG_PCGCCTL_STOPCLK;

	usb_os_delay_us(10);

	/* Device mode configuration */
	reg = USB_DEVICE->DCFG;
	reg &= ~(USB_OTG_DCFG_DESCDMA | USB_OTG_DCFG_PFIVL_0 | USB_OTG_DCFG_PFIVL_1 | USB_OTG_DCFG_ENDEVOUTNAK);
	USB_DEVICE->DCFG = reg;

	if (cfg->speed == USB_SPEED_HIGH_IN_FULL) {
		/* High speed phy in full speed mode */
		(void)usbd_hal_set_device_speed(pcd, USB_SPEED_HIGH_IN_FULL);
	} else if (cfg->speed == USB_SPEED_FULL) {
		/* Full speed phy */
		(void)usbd_hal_set_device_speed(pcd, USB_SPEED_FULL);
	} else {
		/* High speed phy */
		(void)usbd_hal_set_device_speed(pcd, USB_SPEED_HIGH);
	}

	/* Flush the FIFOs */
	if (usb_hal_flush_tx_fifo(0x10U) != HAL_OK) { /* all Tx FIFOs */
		ret = HAL_ERR_PARA;
	}

	if (usb_hal_flush_rx_fifo() != HAL_OK) {
		ret = HAL_ERR_PARA;
	}

	usbd_hal_reset_in_token_queue(pcd);

	/* Clear all pending Device Interrupts */
	USB_DEVICE->DIEPMSK = 0U;
	USB_DEVICE->DOEPMSK = 0U;
	USB_DEVICE->DAINT = 0xFFFFFFFFU;
	USB_DEVICE->DAINTMSK = 0U;

	for (i = 0U; i < USB_MAX_ENDPOINTS; i++) {
		if ((USB_INEP(i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA) {
			if (i == 0U) {
				USB_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_SNAK;
			} else {
				USB_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
			}
		} else {
			USB_INEP(i)->DIEPCTL = 0U;
		}

		USB_INEP(i)->DIEPTSIZ = 0U;
		USB_INEP(i)->DIEPINT  = 0xFFFFU; //0xFB7FU;
		USB_INEP(i)->DIEPDMA  = 0U;
	}

	for (i = 0U; i < USB_MAX_ENDPOINTS; i++) {
		/* FIXME: sgoutnak & epdis+snak & cgoutnak */
		if ((USB_OUTEP(i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA) {
			if (i == 0U) {
				USB_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_SNAK;
			} else {
				USB_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK;
			}
		} else {
			USB_OUTEP(i)->DOEPCTL = 0U;
		}

		USB_OUTEP(i)->DOEPTSIZ = 0U;
		USB_OUTEP(i)->DOEPINT  = 0xFFFFU; //0xFB7FU;
		USB_OUTEP(i)->DOEPDMA  = 0U;
	}

	USB_DEVICE->DIEPMSK &= ~(USB_OTG_DIEPMSK_TXFURM);

	/* Disable all interrupts. */
	USB_GLOBAL->GINTMSK = 0U;

	/* Clear any pending interrupts */
	USB_GLOBAL->GINTSTS = 0xFFFFFFFF; // 0xBFFFFFFFU;

	/* Enable the common interrupts */
	if (cfg->dma_enable == 0U) {
		USB_GLOBAL->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
	}

	/* FIXME: from AmebaD */
	//USB_GLOBAL->GINTMSK |= USB_OTG_GINTMSK_MMISM | USB_OTG_GINTMSK_OTGINT | USB_OTG_GINTMSK_CIDSCHGM | USB_OTG_GINTMSK_SRQIM;
	USB_GLOBAL->GINTMSK |= USB_OTG_GINTMSK_WUIM;

	/* For attach status check */
	//USB_GLOBAL->GINTMSK |= USB_OTG_GINTMSK_ESUSPM;

	/* Enable interrupts matching to the Device mode ONLY */
	USB_GLOBAL->GINTMSK |= USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST |
						   USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IEPINT |
						   USB_OTG_GINTMSK_OEPINT;

	if (cfg->ext_intr_en & USBD_SOF_INTR) {
		USB_GLOBAL->GINTMSK |= USB_OTG_GINTMSK_SOFM;
	}
	if (cfg->ext_intr_en & USBD_EOPF_INTR) {
		USB_GLOBAL->GINTMSK |= USB_OTG_GINTMSK_EOPFM;
	}
	if (cfg->ext_intr_en & USBD_ICII_INTR) {
		USB_GLOBAL->GINTMSK |= USB_OTG_GINTMSK_IISOIXFRM;
	}
	if (cfg->ext_intr_en & USBD_EPMIS_INTR) {
		USB_GLOBAL->GINTMSK |= USB_OTG_GINTMSK_EPMISM;
	}

	return ret;
}

/**
  * @brief  Stop the usb device mode
  * @param  pcd: PCD Instance
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_hal_device_stop(usbd_pcd_t *pcd)
{
	u8 ret;
	u32 i;

	UNUSED(pcd);

	/* Clear Pending interrupt */
	for (i = 0U; i < 15U; i++) {
		USB_INEP(i)->DIEPINT = 0xFFFFU;
		USB_OUTEP(i)->DOEPINT = 0xFFFFU;
	}

	/* Clear interrupt masks */
	USB_GLOBAL->GINTMSK = 0U;
	USB_DEVICE->DIEPMSK  = 0U;
	USB_DEVICE->DOEPMSK  = 0U;
	USB_DEVICE->DAINTMSK = 0U;

	/* Flush the FIFO */
	ret = usb_hal_flush_rx_fifo();
	if (ret != HAL_OK) {
		//DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "usbd_hal_device_stop: usb_hal_flush_rx_fifo fail\n");
		return ret;
	}

	ret = usb_hal_flush_tx_fifo(0x10U);
	if (ret != HAL_OK) {
		//DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "usbd_hal_device_stop: usb_hal_flush_tx_fifo fail\n");
		return ret;
	}

	return ret;
}

/**
  * @brief  Set Tx FIFO
  * @param  pcd: PCD handle
  * @param  fifo: Tx fifo number
  * @param  size: Fifo size
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_hal_set_tx_fifo(usbd_pcd_t *pcd, u8 fifo, u16 size)
{
	UNUSED(pcd);
	u8 i;
	u32 Tx_Offset;

	/*  TXn min size = 16 words. (n  : Transmit FIFO index)
	    When a TxFIFO is not used, the Configuration should be as follows:
	        case 1 :  n > m    and Txn is not used    (n,m  : Transmit FIFO indexes)
	       --> Txm can use the space allocated for Txn.
	       case2  :  n < m    and Txn is not used    (n,m  : Transmit FIFO indexes)
	       --> Txn should be configured with the minimum space of 16 words
	   The FIFO is used optimally when used TxFIFOs are allocated in the top
	       of the FIFO.Ex: use EP1 and EP2 as IN instead of EP1 and EP3 as IN ones.
	   When DMA is used 3n * FIFO locations should be reserved for internal DMA registers */

	Tx_Offset = USB_GLOBAL->GRXFSIZ;

	if (fifo == 0U) {
		USB_GLOBAL->GNPTXFSIZ = ((u32)size << USB_OTG_GNPTXFSIZ_NPTXFDEP_Pos) | Tx_Offset;
	} else {
		Tx_Offset += (USB_GLOBAL->GNPTXFSIZ) >> USB_OTG_GNPTXFSIZ_NPTXFDEP_Pos;
		for (i = 0U; i < (fifo - 1U); i++) {
			Tx_Offset += (USB_GLOBAL->DPTXFSIZ_DIEPTXF[i] >> USB_OTG_DPTXFSIZ_DIEPTXF_TXFD_Pos);
		}

		/* Multiply Tx_Size by 2 to get higher performance */
		USB_GLOBAL->DPTXFSIZ_DIEPTXF[fifo - 1U] = ((u32)size << USB_OTG_DPTXFSIZ_DIEPTXF_TXFD_Pos) | Tx_Offset;
	}

	return HAL_OK;
}

/**
  * @brief  Set Rx FIFO
  * @param  pcd: PCD handle
  * @param  size: Rx fifo size
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_hal_set_rx_fifo(usbd_pcd_t *pcd, u16 size)
{
	UNUSED(pcd);

	USB_GLOBAL->GRXFSIZ = size;

	return HAL_OK;
}

/**
  * @brief  Initializes the DevSpd field of DCFG register
  *         depending the PHY type and the enumeration speed of the device.
  * @param  pcd: PCD Instance
  * @param  speed: Device speed
  *          This parameter can be one of these values:
  *            @arg USB_SPEED_HIGH: High speed mode
  *            @arg USB_SPEED_HIGH_IN_FULL: High speed phy in full speed mode
  *            @arg USB_SPEED_FULL: Full speed mode
  * @retval  Hal status
  */
USB_TEXT_SECTION
u8 usbd_hal_set_device_speed(usbd_pcd_t *pcd, u8 speed)
{
	UNUSED(pcd);
	USB_DEVICE->DCFG |= speed;
	return HAL_OK;
}

/**
  * @brief  Return the Dev Speed
  * @param  pcd: PCD Instance
  * @retval Device speed
  *          This parameter can be one of these values:
  *            @arg USB_SPEED_HIGH: High speed mode
  *            @arg USB_SPEED_HIGH_IN_FULL: High speed phy in full speed mode
  *            @arg USB_SPEED_FULL: Full speed mode
  */
USB_TEXT_SECTION
u8 usbd_hal_get_device_speed(usbd_pcd_t *pcd)
{
	UNUSED(pcd);
	u8 speed;
	u32 DevEnumSpeed = USB_DEVICE->DSTS & USB_OTG_DSTS_ENUMSPD;

	if (DevEnumSpeed == DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ) {
		speed = USB_SPEED_HIGH;
	} else if (DevEnumSpeed == DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ) {
		speed = USB_SPEED_HIGH_IN_FULL;
	} else if (DevEnumSpeed == DSTS_ENUMSPD_FS_PHY_48MHZ) {
		speed = USB_SPEED_FULL;
	} else {
		speed = USB_SPEED_HIGH;
	}

	return speed;
}

/**
  * @brief  Get TxFIFO number
  * @param  pcd: PCD Instance
  * @param  ep: Pointer to endpoint structure
  * @retval TxFIFO number
  *          1 for ISOC EP, using Periodical TxFIFO
  *          0 for other EP, using Non-Periodical TxFIFO
  */
USB_TEXT_SECTION
u32 usbd_hal_get_tx_fifo_num(usbd_pcd_t *pcd, usbd_pcd_ep_t *ep)
{
	u32 tx_fifo_num = 0;

	if (pcd->ded_tx_fifo_en) {
		tx_fifo_num = ep->num;
	} else {
		if ((ep->type == USB_CH_EP_TYPE_ISOC) ||
			((ep->type == USB_CH_EP_TYPE_INTR) && (pcd->config.intr_use_ptx_fifo != 0U))) {
			tx_fifo_num = 1;
		}
	}

	return tx_fifo_num;
}

/**
  * @brief  Activate and configure an endpoint
  * @param  pcd: PCD Instance
  * @param  ep: Pointer to endpoint structure
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_hal_ep_activate(usbd_pcd_t *pcd, usbd_pcd_ep_t *ep)
{
	u32 ep_num = (u32)ep->num;
	u32 tx_fifo_num = 0;
	u32 reg;
	u32 i;
	u32 dcfg;

	if (ep->is_in == 1U) {

		USB_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_IEPM & (u32)(1UL << (ep->num & EP_ADDR_MSK));

		if ((USB_INEP(ep_num)->DIEPCTL & USB_OTG_DIEPCTL_USBAEP) == 0U) {
			tx_fifo_num = usbd_hal_get_tx_fifo_num(pcd, ep);
			reg = USB_INEP(ep_num)->DIEPCTL;
			reg &= ~(USB_OTG_DIEPCTL_MPSIZ_Msk | USB_OTG_DIEPCTL_EPTYP_Msk | USB_OTG_DIEPCTL_TXFNUM_Msk);
			reg |= (ep->max_packet_len & USB_OTG_DIEPCTL_MPSIZ_Msk) |
				   ((u32)ep->type << USB_OTG_DIEPCTL_EPTYP_Pos) | (tx_fifo_num << USB_OTG_DIEPCTL_TXFNUM_Pos) |
				   USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
				   USB_OTG_DIEPCTL_USBAEP;

			if (pcd->config.dma_enable == 1U) {
				for (i = 0; i < USB_MAX_ENDPOINTS; i++) {
					if (pcd->nextep_seq[i] == pcd->first_in_nextep_seq) {
					pcd->nextep_seq[i] = ep->num;
					pcd->nextep_seq[ep->num] = pcd->first_in_nextep_seq;
						break;
					}
				}

				if (ep->type != USB_CH_EP_TYPE_ISOC) {
					reg &= ~USB_OTG_DIEPCTL_NEXTEP_Msk;
					reg |= ((pcd->nextep_seq[ep_num] << USB_OTG_DIEPCTL_NEXTEP_Pos) & USB_OTG_DIEPCTL_NEXTEP_Msk);
				}

				dcfg = USB_DEVICE->DCFG;
				dcfg &= ~USB_OTG_DCFG_EPMISCNT_Msk;
				dcfg |= (USBD_EPMIS_CNT << USB_OTG_DCFG_EPMISCNT_Pos);
				USB_DEVICE->DCFG = dcfg;
			}

			USB_INEP(ep_num)->DIEPCTL = reg;
		}
	} else {
		USB_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_OEPM & ((u32)(1UL << (ep->num & EP_ADDR_MSK)) << USB_OTG_DAINTMSK_OEPM_Pos);

		if (((USB_OUTEP(ep_num)->DOEPCTL) & USB_OTG_DOEPCTL_USBAEP) == 0U) {
			reg = USB_OUTEP(ep_num)->DOEPCTL;
			reg &= ~(USB_OTG_DOEPCTL_MPSIZ_Msk | USB_OTG_DOEPCTL_EPTYP_Msk);
			reg |= (ep->max_packet_len & USB_OTG_DOEPCTL_MPSIZ) |
				   ((u32)ep->type << USB_OTG_DOEPCTL_EPTYP_Pos) |
				   USB_OTG_DOEPCTL_SD0PID_SEVNFRM |
				   USB_OTG_DOEPCTL_USBAEP;
			USB_OUTEP(ep_num)->DOEPCTL = reg;
		}
	}
	return HAL_OK;
}

/**
  * @brief  De-activate and de-initialize an endpoint
  * @param  pcd: PCD Instance
  * @param  ep: Pointer to endpoint structure
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_hal_ep_deactivate(usbd_pcd_t *pcd, usbd_pcd_ep_t *ep)
{
	u32 ep_num = (u32)ep->num;
	u32 reg;
	u32 i;

	/* Read DEPCTLn register */
	if (ep->is_in == 1U) {
		USB_DEVICE->DAINTMSK &= ~(USB_OTG_DAINTMSK_IEPM & (u32)(1UL << (ep->num & EP_ADDR_MSK)));
		reg = USB_INEP(ep_num)->DIEPCTL;
		reg &= ~(USB_OTG_DIEPCTL_USBAEP |
				 USB_OTG_DIEPCTL_MPSIZ |
				 USB_OTG_DIEPCTL_TXFNUM |
				 USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
				 USB_OTG_DIEPCTL_EPTYP);

		if (pcd->config.dma_enable == 1U) {
			for (i = 0; i < USB_MAX_ENDPOINTS; i++) {
				if (pcd->nextep_seq[i] == ep->num) {
					pcd->nextep_seq[i] = pcd->nextep_seq[ep->num];
					if (pcd->first_in_nextep_seq == ep->num) {
						pcd->first_in_nextep_seq = i;
					}
					pcd->nextep_seq[ep->num] = 0xFF;
					break;
				}
			}

			if (ep->type != USB_CH_EP_TYPE_ISOC) {
				reg &= ~USB_OTG_DIEPCTL_NEXTEP_Msk;
			}
		}

		USB_INEP(ep_num)->DIEPCTL = reg;
	} else {
		USB_DEVICE->DAINTMSK &= ~(USB_OTG_DAINTMSK_OEPM & ((u32)(1UL << (ep->num & EP_ADDR_MSK)) << USB_OTG_DAINTMSK_OEPM_Pos));
		USB_OUTEP(ep_num)->DOEPCTL &= ~(USB_OTG_DOEPCTL_USBAEP |
										USB_OTG_DOEPCTL_MPSIZ |
										USB_OTG_DOEPCTL_SD0PID_SEVNFRM |
										USB_OTG_DOEPCTL_EPTYP);
	}

	return HAL_OK;
}

/**
  * @brief  Setup and starts a transfer over an EP
  * @param  pcd: PCD Instance
  * @param  ep: Pointer to endpoint structure
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_hal_ep_start_transfer(usbd_pcd_t *pcd, usbd_pcd_ep_t *ep)
{
	u32 ep_num = (u32)ep->num;
	u32 dma = pcd->config.dma_enable;
	u16 pktcnt;

	/* IN endpoint */
	if (ep->is_in == 1U) {
		/* Zero Length Packet? */
		if (ep->xfer_len == 0U) {
			USB_INEP(ep_num)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);
			USB_INEP(ep_num)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (1U << USB_OTG_DIEPTSIZ_PKTCNT_Pos));
			USB_INEP(ep_num)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
		} else {
			/* Program the transfer size and packet count
			* as follows: xfersize = N * max_packet_len +
			* short_packet pktcnt = N + (short_packet
			* exist ? 1 : 0)
			*/
			USB_INEP(ep_num)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
			USB_INEP(ep_num)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);
			USB_INEP(ep_num)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (((ep->xfer_len + ep->max_packet_len - 1U) / ep->max_packet_len) << USB_OTG_DIEPTSIZ_PKTCNT_Pos));
			USB_INEP(ep_num)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_XFRSIZ & ep->xfer_len);

			if (ep->is_ptx) {
				USB_INEP(ep_num)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_MULCNT);
				USB_INEP(ep_num)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_MULCNT & (1U << USB_OTG_DIEPTSIZ_MULCNT_Pos));
			}
		}

		if (dma == 1U) {
			if ((u32)ep->dma_addr != 0U) {
				USB_INEP(ep_num)->DIEPDMA = (u32)(ep->dma_addr);
			}
			usbd_hal_predict_next_ep(pcd, ep);
		} else {
			if (!(ep->is_ptx)) {
				/* Enable the Tx FIFO Empty Interrupt for this EP */
				if (ep->xfer_len > 0U) {
					if (pcd->ded_tx_fifo_en == 0U) {//share FIFO
						if (!(pcd->config.ext_intr_en & USBD_ITTXFE_INTR)) {
							USB_GLOBAL->GINTMSK |= USB_OTG_GINTMSK_NPTXFEM;
						} else {
							ep->data_to_fifo = 1;
						}
					} else {
						USB_DEVICE->DIEPEMPMSK |= 1UL << (ep_num & EP_ADDR_MSK);
					}
				}
			}
		}

		if (ep->is_ptx) {
			if ((USB_DEVICE->DSTS & (1U << USB_OTG_DSTS_FNSOF_Pos)) == 0U) {
				USB_INEP(ep_num)->DIEPCTL |= USB_OTG_DIEPCTL_SODDFRM;
			} else {
				USB_INEP(ep_num)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
			}
		}

		/* EP enable */
		USB_INEP(ep_num)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);

		if ((ep->is_ptx) && (dma == 0U)) {
			usb_hal_write_packet(ep->xfer_buff, ep->num, (u16)ep->xfer_len);
		}
	} else { /* OUT endpoint */
		/* Program the transfer size and packet count as follows:
		* pktcnt = N
		* xfersize = N * max_packet_len
		*/
		USB_OUTEP(ep_num)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_XFRSIZ);
		USB_OUTEP(ep_num)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_PKTCNT);

		if (ep->xfer_len == 0U) {
			USB_OUTEP(ep_num)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_XFRSIZ & ep->max_packet_len);
			USB_OUTEP(ep_num)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << USB_OTG_DOEPTSIZ_PKTCNT_Pos));
		} else {
			pktcnt = (u16)((ep->xfer_len + ep->max_packet_len - 1U) / ep->max_packet_len);
			USB_OUTEP(ep_num)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_PKTCNT & ((u32)pktcnt << USB_OTG_DOEPTSIZ_PKTCNT_Pos);
			USB_OUTEP(ep_num)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_XFRSIZ & (ep->max_packet_len * pktcnt);
		}

		if (dma) {
			if ((u32)ep->xfer_buff != 0U) {
				USB_OUTEP(ep_num)->DOEPDMA = ep->xfer_buff;
			}
		}

		if (ep->type == USB_CH_EP_TYPE_ISOC) {
			if ((USB_DEVICE->DSTS & (1U << USB_OTG_DSTS_FNSOF_Pos)) == 0U) {
				USB_OUTEP(ep_num)->DOEPCTL |= USB_OTG_DOEPCTL_SODDFRM;
			} else {
				USB_OUTEP(ep_num)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
			}
			//program guide : ISOC OUT Application Flow
			USB_OUTEP(ep_num)->DOEPCTL &= ~(USB_OTG_DOEPCTL_SNAK | USB_OTG_DOEPCTL_EPDIS);
		}
		/* EP enable */
		USB_OUTEP(ep_num)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
	}

	return HAL_OK;
}

/**
  * @brief  Setup and starts a transfer over the EP0
  * @param  pcd: PCD Instance
  * @param  ep: Pointer to endpoint structure
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_hal_ep0_start_transfer(usbd_pcd_t *pcd, usbd_pcd_ep_t *ep)
{
	u32 ep_num = (u32)ep->num;
	u32 dma = pcd->config.dma_enable;

	/* IN endpoint */
	if (ep->is_in == 1U) {
		/* Zero Length Packet? */
		if (ep->xfer_len == 0U) {
			USB_INEP(ep_num)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ0_PKTCNT);
			USB_INEP(ep_num)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ0_PKTCNT & (1U << USB_OTG_DIEPTSIZ0_PKTCNT_Pos));
			USB_INEP(ep_num)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ0_XFRSIZ);
		} else {
			/* Program the transfer size and packet count
			* as follows: xfersize = N * max_packet_len +
			* short_packet pktcnt = N + (short_packet
			* exist ? 1 : 0)
			*/
			USB_INEP(ep_num)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ0_XFRSIZ);
			USB_INEP(ep_num)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ0_PKTCNT);

			if (ep->xfer_len > ep->max_packet_len) {
				ep->xfer_len = ep->max_packet_len;
			}
			USB_INEP(ep_num)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ0_PKTCNT & (1U << USB_OTG_DIEPTSIZ0_PKTCNT_Pos));
			USB_INEP(ep_num)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ0_XFRSIZ & ep->xfer_len);
		}

		if (dma == 1U) {
			if ((u32)ep->dma_addr != 0U) {
				USB_INEP(ep_num)->DIEPDMA = (u32)(ep->dma_addr);
			}
			usbd_hal_predict_next_ep(pcd, ep);
		} else {
			/* Enable the Tx FIFO Empty Interrupt for this EP */
			if (ep->xfer_len > 0U) {
				if (pcd->ded_tx_fifo_en == 0U) {  //share FIFO
					if (!(pcd->config.ext_intr_en & USBD_ITTXFE_INTR)) {
						USB_GLOBAL->GINTMSK |= USB_OTG_GINTMSK_NPTXFEM;
					} else {
						ep->data_to_fifo = 1;
					}
				} else {
					USB_DEVICE->DIEPEMPMSK |= 1UL << (ep_num & EP_ADDR_MSK);
				}
			}
		}

		/* EP enable, IN data in FIFO */
		USB_INEP(ep_num)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
	} else { /* OUT endpoint */
		/* Program the transfer size and packet count as follows:
		* pktcnt = N
		* xfersize = N * max_packet_len
		*/
		USB_OUTEP(ep_num)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ0_XFRSIZ);
		USB_OUTEP(ep_num)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ0_PKTCNT);

		if (ep->xfer_len > 0U) {
			ep->xfer_len = ep->max_packet_len;
		}

		USB_OUTEP(ep_num)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ0_PKTCNT & (1U << 19));
		USB_OUTEP(ep_num)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ0_XFRSIZ & (ep->max_packet_len));

		if (dma == 1U) {
			if ((u32)ep->xfer_buff != 0U) {
				USB_OUTEP(ep_num)->DOEPDMA = (u32)(ep->xfer_buff);
			}
		}

		/* EP enable */
		USB_OUTEP(ep_num)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
	}

	return HAL_OK;
}

/**
  * @brief  Set a stall condition over an EP
  * @param  pcd: PCD Instance
  * @param  ep: Pointer to endpoint structure
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_hal_ep_set_stall(usbd_pcd_t *pcd, usbd_pcd_ep_t *ep)
{
	UNUSED(pcd);
	u32 ep_num = (u32)ep->num;

	if (ep->is_in == 1U) {
		if (((USB_INEP(ep_num)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == 0U) && (ep_num != 0U)) {
			USB_INEP(ep_num)->DIEPCTL &= ~(USB_OTG_DIEPCTL_EPDIS);
		}
		USB_INEP(ep_num)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
	} else {
		if (((USB_OUTEP(ep_num)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == 0U) && (ep_num != 0U)) {
			USB_OUTEP(ep_num)->DOEPCTL &= ~(USB_OTG_DOEPCTL_EPDIS);
		}
		USB_OUTEP(ep_num)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
	}

	return HAL_OK;
}

/**
  * @brief  Clear a stall condition over an EP
  * @param  pcd: PCD Instance
  * @param  ep: Pointer to endpoint structure
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_hal_ep_clear_stall(usbd_pcd_t *pcd, usbd_pcd_ep_t *ep)
{
	UNUSED(pcd);
	u32 ep_num = (u32)ep->num;

	if (ep->is_in == 1U) {
		USB_INEP(ep_num)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
		if ((ep->type == USB_CH_EP_TYPE_INTR) || (ep->type == USB_CH_EP_TYPE_BULK)) {
			USB_INEP(ep_num)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM; /* DATA0 */
		}
	} else {
		USB_OUTEP(ep_num)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
		if ((ep->type == USB_CH_EP_TYPE_INTR) || (ep->type == USB_CH_EP_TYPE_BULK)) {
			USB_OUTEP(ep_num)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM; /* DATA0 */
		}
	}
	return HAL_OK;
}

/**
  * @brief  Set an address to the usb device
  * @param  pcd: PCD Instance
  * @param  address: New device address to be assigned, valid value from 0 to 255
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_hal_set_device_address(usbd_pcd_t *pcd, u8 address)
{
	UNUSED(pcd);
	USB_DEVICE->DCFG &= ~(USB_OTG_DCFG_DAD);
	USB_DEVICE->DCFG |= ((u32)address << 4) & USB_OTG_DCFG_DAD;

	return HAL_OK;
}

/**
  * @brief  Connect the USB device by enabling the pull-up/pull-down
  * @param  pcd: PCD Instance
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_hal_connect(usbd_pcd_t *pcd)
{
	UNUSED(pcd);
	USB_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;
#ifndef CONFIG_USB_FS
	usb_os_delay_ms(100U);
#endif

	return HAL_OK;
}

/**
  * @brief  Disconnect the USB device by disabling the pull-up/pull-down
  * @param  pcd: PCD Instance
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_hal_disconnect(usbd_pcd_t *pcd)
{
	UNUSED(pcd);
	USB_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;
#ifndef CONFIG_USB_FS
	usb_os_delay_ms(3U);
#endif

	return HAL_OK;
}

/**
  * @brief  Return the USB device OUT endpoints interrupt status
  * @param  pcd: PCD Instance
  * @retval OUT endpoints interrupt status
  */
USB_TEXT_SECTION
u32 usbd_hal_read_all_out_ep_interrupts(usbd_pcd_t *pcd)
{
	UNUSED(pcd);
	u32 tmpreg;

	tmpreg  = USB_DEVICE->DAINT;
	tmpreg &= USB_DEVICE->DAINTMSK;

	return ((tmpreg & 0xffff0000U) >> 16);
}

/**
  * @brief  Return the USB device IN endpoints interrupt status
  * @param  pcd: PCD Instance
  * @retval IN endpoints interrupt status
  */
USB_TEXT_SECTION
u32 usbd_hal_read_all_in_ep_interrupts(usbd_pcd_t *pcd)
{
	UNUSED(pcd);
	u32 tmpreg;

	tmpreg  = USB_DEVICE->DAINT;
	tmpreg &= USB_DEVICE->DAINTMSK;

	return ((tmpreg & 0xFFFFU));
}

/**
  * @brief  Returns Device OUT EP Interrupt register
  * @param  pcd: PCD Instance
  * @param  ep_num: Endpoint number, valid value from 0 to 15
  * @retval Device OUT EP Interrupt register
  */
USB_TEXT_SECTION
u32 usbd_hal_read_out_ep_interrupts(usbd_pcd_t *pcd, u8 ep_num)
{
	UNUSED(pcd);
	u32 tmpreg;

	tmpreg  = USB_OUTEP((u32)ep_num)->DOEPINT;
	tmpreg &= USB_DEVICE->DOEPMSK;

	return tmpreg;
}

/**
  * @brief  Returns Device IN EP Interrupt register
  * @param  pcd: PCD Instance
  * @param  ep_num: Endpoint number, valid value from 0 to 15
  * @retval Device IN EP Interrupt register
  */
USB_TEXT_SECTION
u32 usbd_hal_read_in_ep_interrupts(usbd_pcd_t *pcd, u8 ep_num)
{
	UNUSED(pcd);
	u32 reg;
	u32 msk;
	u32 emp_msk;

	msk = USB_DEVICE->DIEPMSK;
	emp_msk = USB_DEVICE->DIEPEMPMSK;
	msk |= ((emp_msk >> (ep_num & EP_ADDR_MSK)) & 0x1U) << USB_OTG_DIEPINT_TXFE_Pos;
	reg = USB_INEP(ep_num)->DIEPINT & msk;

	return reg;
}

/**
  * @brief  Activate EP0 for Setup transactions
  * @param  pcd: PCD Instance
  * @retval HAL status
  */
USB_TEXT_SECTION
u8  usbd_hal_ep0_setup_activate(usbd_pcd_t *pcd)
{
	UNUSED(pcd);

	/* Set the MPS of the IN EP based on the enumeration speed */
	USB_INEP(0U)->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;

	if ((USB_DEVICE->DSTS & USB_OTG_DSTS_ENUMSPD) == DSTS_ENUMSPD_LS_PHY_6MHZ) {
		USB_INEP(0U)->DIEPCTL |= 3U;
	}
	USB_DEVICE->DCTL |= USB_OTG_DCTL_CGINAK;

	return HAL_OK;
}

/**
  * @brief  Prepare the EP0 to start the first control setup
  * @param  pcd: PCD Instance
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_hal_ep0_out_start(usbd_pcd_t *pcd)
{
	u32 gSNPSiD = USB_GLOBAL->GSNPSID;
	u32 dma = pcd->config.dma_enable;
	u8 *setup = (u8 *)pcd->setup;

	if (gSNPSiD > USB_OTG_CORE_ID_300A) {
		if ((USB_OUTEP(0U)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA) {
			return HAL_OK;
		}
	}

	USB_OUTEP(0U)->DOEPTSIZ = 0U;
	USB_OUTEP(0U)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ0_PKTCNT & (1U << 19));
	USB_OUTEP(0U)->DOEPTSIZ |= (3U * 8U);
	USB_OUTEP(0U)->DOEPTSIZ |= USB_OTG_DOEPTSIZ0_STUPCNT;

	if (dma == 1U) {
		usb_os_memset((void *)pcd->setup, 0, USBD_SETUP_PACKET_BUF_LEN);
		DCache_Clean(setup, USBD_SETUP_PACKET_BUF_LEN);
		USB_OUTEP(0U)->DOEPDMA = (u32)setup;
		/* EP enable */
		USB_OUTEP(0U)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_USBAEP;
	}

	return HAL_OK;
}

/**
  * @brief  Handle USB test mode
  * @param  mode: test mode index
  * @retval HAL status
  */
USB_TEXT_SECTION
u8 usbd_hal_test_mode(u8 mode)
{
	u32 reg = USB_DEVICE->DCTL;

	switch (mode) {
	case 1:	// TEST_J
		reg &= ~USB_OTG_DCTL_TCTL_Msk;
		reg |= 1 << USB_OTG_DCTL_TCTL_Pos;
		break;
	case 2:	// TEST_K
		reg &= ~USB_OTG_DCTL_TCTL_Msk;
		reg |= 2 << USB_OTG_DCTL_TCTL_Pos;
		break;
	case 3:	// TEST_SE0_NAK
		reg &= ~USB_OTG_DCTL_TCTL_Msk;
		reg |= 3 << USB_OTG_DCTL_TCTL_Pos;
		break;
	case 4:	// TEST_PACKET
		reg &= ~USB_OTG_DCTL_TCTL_Msk;
		reg |= 4 << USB_OTG_DCTL_TCTL_Pos;
		break;
	case 5:	// TEST_FORCE_ENABLE
		reg &= ~USB_OTG_DCTL_TCTL_Msk;
		reg |= 5 << USB_OTG_DCTL_TCTL_Pos;
		break;
	default:
		break;
	}

	USB_DEVICE->DCTL = reg;

	return HAL_OK;
}

/**
  * @brief  Dump USB registers
  * @param  void
  * @retval HAL status
  */
USB_TEXT_SECTION
void usbd_hal_dump_registers(void)
{
	DiagPrintf("\n*** Core Global CSRs ***\n");
	DiagPrintf("GOTGCTL = 0x%08X\n", USB_GLOBAL->GOTGCTL);
	DiagPrintf("GOTGINT = 0x%08X\n", USB_GLOBAL->GOTGINT);
	DiagPrintf("GAHBCFG = 0x%08X\n", USB_GLOBAL->GAHBCFG);
	DiagPrintf("GUSBCFG = 0x%08X\n", USB_GLOBAL->GUSBCFG);
	DiagPrintf("GRSTCTL = 0x%08X\n", USB_GLOBAL->GRSTCTL);
	DiagPrintf("GINTSTS = 0x%08X\n", USB_GLOBAL->GINTSTS);
	DiagPrintf("GINTMSK = 0x%08X\n", USB_GLOBAL->GINTMSK);
	DiagPrintf("GRXSTSR = 0x%08X\n", USB_GLOBAL->GRXSTSR);
	DiagPrintf("GRXSTSP = 0x%08X\n", USB_GLOBAL->GRXSTSP);
	DiagPrintf("GRXFSIZ = 0x%08X\n", USB_GLOBAL->GRXFSIZ);
	DiagPrintf("GNPTXFSIZ = 0x%08X\n", USB_GLOBAL->GNPTXFSIZ);
	DiagPrintf("GNPTXSTS = 0x%08X\n", USB_GLOBAL->GNPTXSTS);
	DiagPrintf("GPVNDCTL = 0x%08X\n", USB_GLOBAL->GPVNDCTL);
	DiagPrintf("GUID = 0x%08X\n", USB_GLOBAL->GUID);
	DiagPrintf("GSNPSID = 0x%08X\n", USB_GLOBAL->GSNPSID);
	DiagPrintf("GHWCFG1 = 0x%08X\n", USB_GLOBAL->GHWCFG1);
	DiagPrintf("GHWCFG2 = 0x%08X\n", USB_GLOBAL->GHWCFG2);
	DiagPrintf("GHWCFG3 = 0x%08X\n", USB_GLOBAL->GHWCFG3);
	DiagPrintf("GHWCFG4 = 0x%08X\n", USB_GLOBAL->GHWCFG4);
	DiagPrintf("GLPMCFG = 0x%08X\n", USB_GLOBAL->GLPMCFG);
	DiagPrintf("GPWRDN = 0x%08X\n", USB_GLOBAL->GPWRDN);
	DiagPrintf("DPTXFSIZ_DIEPTXF[0] = 0x%08X\n", USB_GLOBAL->DPTXFSIZ_DIEPTXF[0]);
	DiagPrintf("DPTXFSIZ_DIEPTXF[1] = 0x%08X\n", USB_GLOBAL->DPTXFSIZ_DIEPTXF[1]);
	DiagPrintf("DPTXFSIZ_DIEPTXF[2] = 0x%08X\n", USB_GLOBAL->DPTXFSIZ_DIEPTXF[2]);
	DiagPrintf("DPTXFSIZ_DIEPTXF[3] = 0x%08X\n", USB_GLOBAL->DPTXFSIZ_DIEPTXF[3]);
	DiagPrintf("DPTXFSIZ_DIEPTXF[4] = 0x%08X\n", USB_GLOBAL->DPTXFSIZ_DIEPTXF[4]);
	DiagPrintf("DPTXFSIZ_DIEPTXF[5] = 0x%08X\n", USB_GLOBAL->DPTXFSIZ_DIEPTXF[5]);
	DiagPrintf("DPTXFSIZ_DIEPTXF[6] = 0x%08X\n", USB_GLOBAL->DPTXFSIZ_DIEPTXF[6]);
	DiagPrintf("DPTXFSIZ_DIEPTXF[7] = 0x%08X\n", USB_GLOBAL->DPTXFSIZ_DIEPTXF[7]);
	DiagPrintf("\n*** Device Mode CSRs ***\n");
	DiagPrintf("DCFG = 0x%08X\n", USB_DEVICE->DCFG);
	DiagPrintf("DCTL = 0x%08X\n", USB_DEVICE->DCTL);
	DiagPrintf("DSTS = 0x%08X\n", USB_DEVICE->DSTS);
	DiagPrintf("DIEPMSK = 0x%08X\n", USB_DEVICE->DIEPMSK);
	DiagPrintf("DOEPMSK = 0x%08X\n", USB_DEVICE->DOEPMSK);
	DiagPrintf("DAINT = 0x%08X\n", USB_DEVICE->DAINT);
	DiagPrintf("DAINTMSK = 0x%08X\n", USB_DEVICE->DAINTMSK);
	DiagPrintf("DTKNQR1 = 0x%08X\n", USB_DEVICE->DTKNQR1);
	DiagPrintf("DTKNQR2 = 0x%08X\n", USB_DEVICE->DTKNQR2);
	DiagPrintf("DVBUSDIS = 0x%08X\n", USB_DEVICE->DVBUSDIS);
	DiagPrintf("DVBUSPULSE = 0x%08X\n", USB_DEVICE->DVBUSPULSE);
	DiagPrintf("DTKNQR3 = 0x%08X\n", USB_DEVICE->DTKNQR3);
	DiagPrintf("DTKNQR4/DIEPEMPMSK = 0x%08X\n", USB_DEVICE->DIEPEMPMSK);
	DiagPrintf("DIEPCTL[0] = 0x%08X\n", USB_INEP(0U)->DIEPCTL);
	DiagPrintf("DIEPINT[0] = 0x%08X\n", USB_INEP(0U)->DIEPINT);
	DiagPrintf("DIEPTSIZ[0] = 0x%08X\n", USB_INEP(0U)->DIEPTSIZ);
	DiagPrintf("DIEPDMA[0] = 0x%08X\n", USB_INEP(0U)->DIEPDMA);
	DiagPrintf("DTXFSTS[0] = 0x%08X\n", USB_INEP(0U)->DTXFSTS);
	DiagPrintf("DIEPDMAB[0] = 0x%08X\n", USB_INEP(0U)->DIEPDMAB);
	DiagPrintf("DIEPCTL[1] = 0x%08X\n", USB_INEP(1U)->DIEPCTL);
	DiagPrintf("DIEPINT[1] = 0x%08X\n", USB_INEP(1U)->DIEPINT);
	DiagPrintf("DIEPTSIZ[1] = 0x%08X\n", USB_INEP(1U)->DIEPTSIZ);
	DiagPrintf("DIEPDMA[1] = 0x%08X\n", USB_INEP(1U)->DIEPDMA);
	DiagPrintf("DTXFSTS[1] = 0x%08X\n", USB_INEP(1U)->DTXFSTS);
	DiagPrintf("DIEPDMAB[1] = 0x%08X\n", USB_INEP(1U)->DIEPDMAB);
	DiagPrintf("DIEPCTL[3] = 0x%08X\n", USB_INEP(3U)->DIEPCTL);
	DiagPrintf("DIEPINT[3] = 0x%08X\n", USB_INEP(3U)->DIEPINT);
	DiagPrintf("DIEPTSIZ[3] = 0x%08X\n", USB_INEP(3U)->DIEPTSIZ);
	DiagPrintf("DIEPDMA[3] = 0x%08X\n", USB_INEP(3U)->DIEPDMA);
	DiagPrintf("DTXFSTS[3] = 0x%08X\n", USB_INEP(3U)->DTXFSTS);
	DiagPrintf("DIEPDMAB[3] = 0x%08X\n", USB_INEP(3U)->DIEPDMAB);
	DiagPrintf("DOEPCTL[0] = 0x%08X\n", USB_OUTEP(0U)->DOEPCTL);
	DiagPrintf("DOEPINT[0] = 0x%08X\n", USB_OUTEP(0U)->DOEPINT);
	DiagPrintf("DOEPTSIZ[0] = 0x%08X\n", USB_OUTEP(0U)->DOEPTSIZ);
	DiagPrintf("DOEPDMA[0] = 0x%08X\n", USB_OUTEP(0U)->DOEPDMA);
	DiagPrintf("DOEPDMAB[0] = 0x%08X\n", USB_OUTEP(0U)->DOEPDMAB);
	DiagPrintf("DOEPCTL[2] = 0x%08X\n", USB_OUTEP(2U)->DOEPCTL);
	DiagPrintf("DOEPINT[2] = 0x%08X\n", USB_OUTEP(2U)->DOEPINT);
	DiagPrintf("DOEPTSIZ[2] = 0x%08X\n", USB_OUTEP(2U)->DOEPTSIZ);
	DiagPrintf("DOEPDMA[2] = 0x%08X\n", USB_OUTEP(2U)->DOEPDMA);
	DiagPrintf("DOEPDMAB[2] = 0x%08X\n", USB_OUTEP(2U)->DOEPDMAB);
}

