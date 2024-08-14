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

USB_DATA_SECTION
static const char *const TAG = "USB";

/* Private functions ---------------------------------------------------------*/

#if (defined(CONFIG_AMEBAD2) || defined(CONFIG_AMEBADPLUS) || defined(CONFIG_RTL8721D) || defined (CONFIG_RTL8720F))

/**
  * @brief  Set Tx FIFO
  * @param  pcd: PCD handle
  * @param  fifo: Tx fifo number
  * @param  size: Fifo size
  * @retval HAL status
  */
USB_TEXT_SECTION
static u8 usbd_hal_set_tx_fifo(usbd_pcd_t *pcd, u8 fifo, u16 size)
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

#endif

#if (defined(CONFIG_AMEBAD2) || defined(CONFIG_AMEBADPLUS) || defined(CONFIG_RTL8721D))

/**
  * @brief  Set Rx FIFO
  * @param  pcd: PCD handle
  * @param  size: Rx fifo size
  * @retval HAL status
  */
USB_TEXT_SECTION
static u8 usbd_hal_set_rx_fifo(usbd_pcd_t *pcd, u16 size)
{
	UNUSED(pcd);

	USB_GLOBAL->GRXFSIZ = size;

	return HAL_OK;
}

#endif

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
	u32 ep_num = USB_EP_NUM(ep->addr);

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

	if (pcd->config.dma_enable) {
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
		usbd_hal_set_device_speed(pcd, USB_SPEED_HIGH_IN_FULL);
	} else if (cfg->speed == USB_SPEED_FULL) {
		/* Full speed phy */
		usbd_hal_set_device_speed(pcd, USB_SPEED_FULL);
	} else {
		/* High speed phy */
		usbd_hal_set_device_speed(pcd, USB_SPEED_HIGH);
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
		RTK_LOGE(TAG, "Fail to flush RX FIFO\n");
		return ret;
	}

	ret = usb_hal_flush_tx_fifo(0x10U);
	if (ret != HAL_OK) {
		RTK_LOGE(TAG, "Fail to flush TX FIFO\n");
		return ret;
	}

	return ret;
}

/**
  * @brief  Configure DFIFO.
  * @param  pcd: PCD Instance
  * @retval Status
  */
USB_TEXT_SECTION
u8 usbd_hal_config_dfifo(usbd_pcd_t *pcd)
{
#if defined (CONFIG_RTL8720F)
	usbd_hal_set_tx_fifo(pcd, 1, 16);
	usbd_hal_set_tx_fifo(pcd, 2, 256);
	usbd_hal_set_tx_fifo(pcd, 3, 32);
	usbd_hal_set_tx_fifo(pcd, 4, 256);
#elif (defined(CONFIG_AMEBAD2) || defined(CONFIG_AMEBADPLUS) || defined(CONFIG_RTL8721D))
	u8 auto_adjust_dfifo_depth = 0U;
	usbd_config_t *cfg = &pcd->config;
	u32 max_rx_fifo_depth = (USB_GLOBAL->GRXFSIZ & USB_OTG_GRXFSIZ_RXFD_Msk) >> USB_OTG_GRXFSIZ_RXFD_Pos;
	u32 max_nptx_fifo_depth = (USB_GLOBAL->GNPTXFSIZ & USB_OTG_GNPTXFSIZ_NPTXFDEP_Msk) >> USB_OTG_GNPTXFSIZ_NPTXFDEP_Pos;
	u32 max_ptx_fifo_depth = (USB_GLOBAL->DPTXFSIZ_DIEPTXF[0] & USB_OTG_DPTXFSIZ_DIEPTXF_TXFD_Msk) >> USB_OTG_DPTXFSIZ_DIEPTXF_TXFD_Pos;
	u32 max_fifo_depth = (USB_GLOBAL->GHWCFG3 & USB_OTG_GHWCFG3_DFIFODEP_Msk) >> USB_OTG_GHWCFG3_DFIFODEP_Pos;

	if ((cfg->rx_fifo_depth  == 0U) && (cfg->nptx_fifo_depth  == 0U) && (cfg->ptx_fifo_depth  == 0U)) {
		RTK_LOGI(TAG, "DFIFO size not specified, use default value\n");
		auto_adjust_dfifo_depth = 1U;
		cfg->rx_fifo_depth = max_rx_fifo_depth;
		cfg->nptx_fifo_depth = max_nptx_fifo_depth;
		cfg->ptx_fifo_depth = max_fifo_depth - max_rx_fifo_depth - max_nptx_fifo_depth;
	} else {
		if (cfg->rx_fifo_depth > max_rx_fifo_depth) {
			RTK_LOGE(TAG, "Invalid rx_fifo_depth value %d, allowed max value %d\n", cfg->rx_fifo_depth, max_rx_fifo_depth);
			return HAL_ERR_PARA;
		} else if (cfg->rx_fifo_depth  == 0U) {
			auto_adjust_dfifo_depth = 1U;
			cfg->rx_fifo_depth = max_rx_fifo_depth;
			RTK_LOGW(TAG, "No rx_fifo_depth specified, use default value %d\n", max_rx_fifo_depth);
		}

		if (cfg->nptx_fifo_depth > max_nptx_fifo_depth) {
			RTK_LOGE(TAG, "Invalid nptx_fifo_depth value %d, allowed max value %d\n", cfg->nptx_fifo_depth, max_nptx_fifo_depth);
			return HAL_ERR_PARA;
		} else if (cfg->nptx_fifo_depth  == 0U) {
			auto_adjust_dfifo_depth = 1U;
			cfg->nptx_fifo_depth = max_nptx_fifo_depth;
			RTK_LOGW(TAG, "No nptx_fifo_depth specified, use default value %d\n", max_nptx_fifo_depth);
		}

		if (cfg->ptx_fifo_depth > max_ptx_fifo_depth) {
			RTK_LOGE(TAG, "Invalid ptx_fifo_depth value %d, allowed max value %d\n", cfg->rx_fifo_depth, max_rx_fifo_depth);
			return HAL_ERR_PARA;
		} else if (cfg->ptx_fifo_depth  == 0U) {
			auto_adjust_dfifo_depth = 1U;
			cfg->ptx_fifo_depth = max_ptx_fifo_depth;
			RTK_LOGW(TAG, "No ptx_fifo_depth specified, use default value %d\n", max_ptx_fifo_depth);
		}

		if (cfg->rx_fifo_depth + cfg->nptx_fifo_depth + cfg->ptx_fifo_depth > max_fifo_depth) {
			RTK_LOGE(TAG, "Invalid DFIFO configuration, please limit the max total DFIFO size to %d\n", max_fifo_depth);
			return HAL_ERR_PARA;
		}
	}

	if (auto_adjust_dfifo_depth) {
		RTK_LOGI(TAG, "DFIFO configurations:\n");
		RTK_LOGI(TAG, "* rx_fifo_depth: %d\n", cfg->rx_fifo_depth);
		RTK_LOGI(TAG, "* nptx_fifo_depth: %d\n", cfg->nptx_fifo_depth);
		RTK_LOGI(TAG, "* ptx_fifo_depth: %d\n", cfg->ptx_fifo_depth);
	}

	usbd_hal_set_rx_fifo(pcd, cfg->rx_fifo_depth);
	usbd_hal_set_tx_fifo(pcd, 0, cfg->nptx_fifo_depth);
	usbd_hal_set_tx_fifo(pcd, 1, cfg->ptx_fifo_depth);
#else
	UNUSED(pcd);
#endif

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
		tx_fifo_num = USB_EP_NUM(ep->addr);
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
	u8 ep_num = USB_EP_NUM(ep->addr);
	u32 tx_fifo_num = 0;
	u32 reg;
	u32 i;
	u32 dcfg;

	if (USB_EP_IS_IN(ep->addr)) {
		USB_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_IEPM & (u32)(1UL << ep_num);

		if ((USB_INEP(ep_num)->DIEPCTL & USB_OTG_DIEPCTL_USBAEP) == 0U) {
			tx_fifo_num = usbd_hal_get_tx_fifo_num(pcd, ep);
			reg = USB_INEP(ep_num)->DIEPCTL;
			reg &= ~(USB_OTG_DIEPCTL_MPSIZ_Msk | USB_OTG_DIEPCTL_EPTYP_Msk | USB_OTG_DIEPCTL_TXFNUM_Msk);
			reg |= (ep->max_packet_len & USB_OTG_DIEPCTL_MPSIZ_Msk) |
				   ((u32)ep->type << USB_OTG_DIEPCTL_EPTYP_Pos) | (tx_fifo_num << USB_OTG_DIEPCTL_TXFNUM_Pos) |
				   USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
				   USB_OTG_DIEPCTL_USBAEP;

			if (pcd->config.dma_enable) {
				for (i = 0; i < USB_MAX_ENDPOINTS; i++) {
					if (pcd->nextep_seq[i] == pcd->first_in_nextep_seq) {
						pcd->nextep_seq[i] = ep_num;
						pcd->nextep_seq[ep_num] = pcd->first_in_nextep_seq;
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
		USB_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_OEPM & ((u32)(1UL << ep_num) << USB_OTG_DAINTMSK_OEPM_Pos);

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
	u8 ep_num = USB_EP_NUM(ep->addr);
	u32 reg;
	u32 i;

	/* Read DEPCTLn register */
	if (USB_EP_IS_IN(ep->addr)) {
		USB_DEVICE->DAINTMSK &= ~(USB_OTG_DAINTMSK_IEPM & (u32)(1UL << ep_num));
		reg = USB_INEP(ep_num)->DIEPCTL;
		reg &= ~(USB_OTG_DIEPCTL_USBAEP |
				 USB_OTG_DIEPCTL_MPSIZ |
				 USB_OTG_DIEPCTL_TXFNUM |
				 USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
				 USB_OTG_DIEPCTL_EPTYP);

		if (pcd->config.dma_enable) {
			for (i = 0; i < USB_MAX_ENDPOINTS; i++) {
				if (pcd->nextep_seq[i] == ep_num) {
					pcd->nextep_seq[i] = pcd->nextep_seq[ep_num];
					if (pcd->first_in_nextep_seq == ep_num) {
						pcd->first_in_nextep_seq = i;
					}
					pcd->nextep_seq[ep_num] = 0xFF;
					break;
				}
			}

			if (ep->type != USB_CH_EP_TYPE_ISOC) {
				reg &= ~USB_OTG_DIEPCTL_NEXTEP_Msk;
			}
		}

		USB_INEP(ep_num)->DIEPCTL = reg;
	} else {
		USB_DEVICE->DAINTMSK &= ~(USB_OTG_DAINTMSK_OEPM & ((u32)(1UL << ep_num) << USB_OTG_DAINTMSK_OEPM_Pos));
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
	u8 ep_num = USB_EP_NUM(ep->addr);
	u8 dma = pcd->config.dma_enable;
	u32 pktcnt;

	/* IN endpoint */
	if (USB_EP_IS_IN(ep->addr)) {
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
			USB_INEP(ep_num)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_XFRSIZ & (ep->xfer_len << USB_OTG_DIEPTSIZ_XFRSIZ_Pos));

			if (ep->is_ptx) {
				USB_INEP(ep_num)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_MULCNT);
				USB_INEP(ep_num)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_MULCNT & (1U << USB_OTG_DIEPTSIZ_MULCNT_Pos));
			}
		}

		if (dma) {
			if (ep->dma_addr != 0U) {
				USB_INEP(ep_num)->DIEPDMA = ep->dma_addr;
			}
			usbd_hal_predict_next_ep(pcd, ep);
		} else {
			if (!(ep->is_ptx)) {
				/* Enable the Tx FIFO Empty Interrupt for this EP */
				if (ep->xfer_len > 0U) {
					if (pcd->ded_tx_fifo_en == 0U) {
						USB_GLOBAL->GINTMSK |= USB_OTG_GINTMSK_NPTXFEM;
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
			usb_hal_write_packet(ep->xfer_buff, ep_num, (u16)ep->xfer_len);
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
			pktcnt = (ep->xfer_len + ep->max_packet_len - 1U) / ep->max_packet_len;
			USB_OUTEP(ep_num)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_PKTCNT & (pktcnt << USB_OTG_DOEPTSIZ_PKTCNT_Pos);
			USB_OUTEP(ep_num)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_XFRSIZ & ((ep->max_packet_len * pktcnt) << USB_OTG_DOEPTSIZ_XFRSIZ_Pos);
		}

		if (dma) {
			if (ep->dma_addr != 0U) {
				USB_OUTEP(ep_num)->DOEPDMA = ep->dma_addr;
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
	u8 ep_num = USB_EP_NUM(ep->addr);
	u8 dma = pcd->config.dma_enable;

	/* IN endpoint */
	if (USB_EP_IS_IN(ep->addr)) {
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

		if (dma) {
			if (ep->dma_addr != 0U) {
				USB_INEP(ep_num)->DIEPDMA = ep->dma_addr;
			}
			usbd_hal_predict_next_ep(pcd, ep);
		} else {
			/* Enable the Tx FIFO Empty Interrupt for this EP */
			if (ep->xfer_len > 0U) {
				if (pcd->ded_tx_fifo_en == 0U) {
					USB_GLOBAL->GINTMSK |= USB_OTG_GINTMSK_NPTXFEM;
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

		USB_OUTEP(ep_num)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ0_PKTCNT & (1U << USB_OTG_DOEPTSIZ0_PKTCNT_Pos));
		USB_OUTEP(ep_num)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ0_XFRSIZ & (ep->max_packet_len << USB_OTG_DOEPTSIZ0_XFRSIZ_Pos));

		if (dma) {
			if (ep->dma_addr != 0U) {
				USB_OUTEP(ep_num)->DOEPDMA = ep->dma_addr;
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
	u8 ep_num = USB_EP_NUM(ep->addr);

	if (USB_EP_IS_IN(ep->addr)) {
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
	u8 ep_num = USB_EP_NUM(ep->addr);

	if (USB_EP_IS_IN(ep->addr)) {
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
	usb_os_sleep_ms(100U);
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
	usb_os_sleep_ms(3U);
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
	u32 setup = (u32)pcd->setup;

	if (gSNPSiD > USB_OTG_CORE_ID_300A) {
		if ((USB_OUTEP(0U)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA) {
			return HAL_OK;
		}
	}

	USB_OUTEP(0U)->DOEPTSIZ = 0U;
	USB_OUTEP(0U)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ0_PKTCNT & (1U << 19));
	USB_OUTEP(0U)->DOEPTSIZ |= (3U * 8U);
	USB_OUTEP(0U)->DOEPTSIZ |= USB_OTG_DOEPTSIZ0_STUPCNT;

	if (dma) {
		_memset((void *)pcd->setup, 0, USBD_SETUP_PACKET_BUF_LEN);
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
  * @brief  Get the bus status
  * @param  pcd: PCD handle
  * @param  bus_status: bus status
  * @retval status
  */
USB_TEXT_SECTION
u8 usbd_hal_get_bus_status(usbd_pcd_t *pcd, u32 *bus_status)
{
	u32 result;
	u32 dsts;
	UNUSED(pcd);

	dsts = USB_DEVICE->DSTS;
	result = (dsts & USB_OTG_DSTS_DEVLNSTS_Msk) >> USB_OTG_DSTS_DEVLNSTS_Pos;
	result |= (dsts & USB_OTG_DSTS_SUSPSTS) ? USBD_BUS_STATUS_SUSPEND : 0;

	*bus_status = result;

	return HAL_OK;
}

/**
  * @brief  Device send a remote wakeup signal to host
  * @param  pcd: PCD handle
  * @retval status
  */
USB_TEXT_SECTION
u8 usbd_hal_wake_host(usbd_pcd_t *pcd)
{
	u32 dsts;

	usb_os_spinlock(pcd->lock);

	dsts = USB_DEVICE->DCTL & ~USB_OTG_DCTL_RWUSIG;
	dsts |= USB_OTG_DCTL_RWUSIG;
	USB_DEVICE->DCTL = dsts;
	/*
	    As specified in the USB 2.0 specification,
	    the application must clear this bit 1~15 ms after setting it.
	*/
	usb_os_delay_us(2000);//delay 2ms,not release cpu

	dsts &= ~USB_OTG_DCTL_RWUSIG;
	USB_DEVICE->DCTL = dsts;

	usb_os_spinunlock(pcd->lock);

	return HAL_OK;
}

