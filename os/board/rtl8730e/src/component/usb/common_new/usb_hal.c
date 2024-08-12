/**
  ******************************************************************************
  * @file    usb_hal.c
  * @author  Realsil WLAN5 Team
  * @brief   This file provides the common USB HAL API
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

#include "ameba_soc.h"
#include "usb_os.h"
#include "usb_hal.h"

/* Private defines -----------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Reset the USB Core (needed after USB clock settings change)
  * @retval Status
  */
USB_TEXT_SECTION
static u8 usb_hal_core_reset(void)
{
	u32 count = 0U;
	u32 gSNPSiD = USB_GLOBAL->GSNPSID;

	DBG_PRINTF(MODULE_USB_OTG, LEVEL_INFO, "GSNPSID = 0x%08X\n", gSNPSiD);

	/* Wait for AHB master IDLE state. */
	do {
		usb_os_delay_us(10U);
		if (++count > 100000U) {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "usb_hal_core_reset: TO1 0x%08X\n", USB_GLOBAL->GRSTCTL);
			return HAL_TIMEOUT;
		}
	} while ((USB_GLOBAL->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);

	/* Core Soft Reset */
	count = 0U;
	USB_GLOBAL->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;

	if (gSNPSiD < USB_OTG_CORE_ID_420A) {
		do {
			if (++count > 10000U) {
				DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "usb_hal_core_reset: TO2 0x%08X\n", USB_GLOBAL->GRSTCTL);
				return HAL_TIMEOUT;
			}
			usb_os_delay_us(1U);
		} while ((USB_GLOBAL->GRSTCTL & USB_OTG_GRSTCTL_CSRST) == USB_OTG_GRSTCTL_CSRST);
	} else {
		do {
			if (++count > 10000U) {
				DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "usb_hal_core_reset: TO2 0x%08X\n", USB_GLOBAL->GRSTCTL);
				return HAL_TIMEOUT;
			}
			usb_os_delay_us(1U);
		} while ((USB_GLOBAL->GRSTCTL & USB_OTG_GRSTCTL_CSRSTDONE) == 0U);

		USB_GLOBAL->GRSTCTL &= ~USB_OTG_GRSTCTL_CSRST;
		USB_GLOBAL->GRSTCTL |= USB_OTG_GRSTCTL_CSRSTDONE;
	}

#ifndef CONFIG_USB_FS
	usb_os_delay_ms(100);
#endif

	return HAL_OK;
}

#if CONFIG_USB_PHY

/**
  * @brief  Load PHY vendor control registers
  * @param  addr: PHY register address
  * @retval Status
  */
USB_TEXT_SECTION
static u8 usb_hal_load_phy_vendor_control_register(u8 addr)
{
	u8 ret = HAL_OK;
	u32 count = 0U;
	u32 reg = 0x0A300000U;

	reg |= (((u32)(USB_OTG_PHY_LOW_ADDR(addr))) << USB_OTG_GPVNDCTL_VCTRL_Pos);
	USB_GLOBAL->GPVNDCTL = reg;

	do {
		/* 1us timeout expected, 1ms for safe */
		DelayUs(1);
		if (++count > 1000U) {
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "Fail to load phy vendor control register\n");
			ret = HAL_TIMEOUT;
			break;
		}
	} while ((USB_GLOBAL->GPVNDCTL & USB_OTG_GPVNDCTL_VSTSDONE) == 0U);

	return ret;
}

/**
  * @brief  Select USB PHY register page
  * @param  None
  * @retval Status
  */
USB_TEXT_SECTION
static u8 usb_hal_select_phy_register_page(u8 page)
{
	u8 ret;
	u8 reg;

	ret = usb_hal_read_phy_register(USB_OTG_PHY_REG_F4, &reg);
	if (ret != HAL_OK) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "Fail to read USB_OTG_PHY_REG_F4: %d\n", ret);
		return ret;
	}

	reg &= (~USB_OTG_PHY_REG_F4_BIT_PAGE_SEL_MASK);
	reg |= (page << USB_OTG_PHY_REG_F4_BIT_PAGE_SEL_POS) & USB_OTG_PHY_REG_F4_BIT_PAGE_SEL_MASK;
	ret = usb_hal_write_phy_register(USB_OTG_PHY_REG_F4, reg);
	if (ret != HAL_OK) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "Fail to write USB_OTG_PHY_REG_F4: %d\n", ret);
	}

	return ret;
}

#endif

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initializes the USB Core
  * @param  dma_enable DMA enable flag
  * @retval Status
  */
USB_TEXT_SECTION
u8 usb_hal_core_init(u8 dma_enable)
{
	u8 ret = HAL_OK;

	USB_GLOBAL->GUSBCFG &= ~USB_OTG_GUSBCFG_ULPIUTMISEL;
	USB_GLOBAL->GUSBCFG |= USB_OTG_GUSBCFG_PHYIF;

	ret = usb_hal_core_reset();
	if (ret != HAL_OK) {
		return ret;
	}

	if (dma_enable == 1U) {
		USB_GLOBAL->GAHBCFG |= (USB_OTG_GAHBCFG_HBSTLEN_2 | USB_OTG_GAHBCFG_DMAEN);
	}

	USB_GLOBAL->GOTGCTL &= ~USB_OTG_GOTGCTL_OTGVER;

	return ret;
}

/**
  * @brief  Enable USB interrupt
  * @retval void
  */
USB_TEXT_SECTION
void usb_hal_enable_interrupt(void)
{
	InterruptEn(USB_IRQ, USB_IRQ_PRI);
}

/**
  * @brief  Disable USB interrupt
  * @retval void
  */
USB_TEXT_SECTION
void usb_hal_disable_interrupt(void)
{
	InterruptDis(USB_IRQ);
}

/**
  * @brief  Register USB IRQ handler
  * @param  handler: IRQ handler
  * @retval void
  */
USB_TEXT_SECTION
void usb_hal_register_irq_handler(usb_irq_fun_t handler)
{
	if (handler != NULL) {
		InterruptRegister(handler, USB_IRQ, (int)NULL, USB_IRQ_PRI);
	}
}

/**
  * @brief  Unregister USB IRQ handler
  * @retval void
  */
USB_TEXT_SECTION
void usb_hal_unregister_irq_handler(void)
{
	InterruptUnRegister(USB_IRQ);
}

/**
  * @brief  Enables the controller's Global Int in the AHB Config reg
  * @retval void
  */
USB_TEXT_SECTION
void usb_hal_enable_global_interrupt(void)
{
	USB_GLOBAL->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
}

/**
  * @brief  Disable the controller's Global Int in the AHB Config reg
  * @retval void
*/
USB_TEXT_SECTION
void usb_hal_disable_global_interrupt(void)
{
	USB_GLOBAL->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
}

/**
  * @brief  Return the global USB interrupt status
  * @retval Global USB interrupt status
  */
USB_TEXT_SECTION
u32 usb_hal_read_interrupts(void)
{
	u32 reg;

	reg = USB_GLOBAL->GINTSTS;
	reg &= USB_GLOBAL->GINTMSK;

	return reg;
}

/**
  * @brief  Clear a USB interrupt
  * @param  interrupt  interrupt flag
  * @retval void
  */
USB_TEXT_SECTION
void usb_hal_clear_interrupts(u32 interrupt)
{
	USB_GLOBAL->GINTSTS |= interrupt;
}

/**
  * @brief  Set functional mode
  * @param  mode OTG mode
  * @retval Status
  */
USB_TEXT_SECTION
u8 usb_hal_set_otg_mode(usb_otg_mode_t mode)
{
	USB_GLOBAL->GUSBCFG &= ~(USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD);

	if (mode == USB_OTG_MODE_DEVICE) {
		USB_GLOBAL->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
	} else if (mode == USB_OTG_MODE_HOST) {
#if CONFIG_USB_OTG
		USB_GLOBAL->GUSBCFG &= ~USB_OTG_GUSBCFG_FHMOD;
#else
		USB_GLOBAL->GUSBCFG |= USB_OTG_GUSBCFG_FHMOD;
#endif
	} else {
		return HAL_ERR_PARA;
	}

#ifndef CONFIG_USB_FS
	usb_os_delay_ms(100U);
#endif

	return HAL_OK;
}

/**
  * @brief  Returns USB core mode
  * @retval return core mode : Host or Device
  *         This parameter can be one of these values:
  *         0 : Host
  *         1 : Device
  */
USB_TEXT_SECTION
usb_otg_mode_t usb_hal_get_otg_mode(void)
{
	usb_otg_mode_t mode = USB_OTG_MODE_DEVICE;
	if (((USB_GLOBAL->GINTSTS) & 0x1U) != 0U) {
		mode = USB_OTG_MODE_HOST;
	}

	return mode;
}

/**
  * @brief  Flush a Tx FIFO
  * @param  num  FIFO number
  *         This parameter can be a value from 1 to 15
            15 means Flush all Tx FIFOs
  * @retval Status
  */
USB_TEXT_SECTION
u8 usb_hal_flush_tx_fifo(u32 num)
{
	u32 count = 0U;

	USB_GLOBAL->GRSTCTL = (USB_OTG_GRSTCTL_TXFFLSH | (num << USB_OTG_GRSTCTL_TXFNUM_Pos));

	do {
		if (++count > 200000U) {
			return HAL_TIMEOUT;
		}
		usb_os_delay_us(1U);
	} while ((USB_GLOBAL->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH);
	usb_os_delay_us(1U);

	return HAL_OK;
}

/**
  * @brief  Flush Rx FIFO
  * @retval Status
  */
USB_TEXT_SECTION
u8 usb_hal_flush_rx_fifo(void)
{
	u32 count = 0;

	USB_GLOBAL->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;

	do {
		if (++count > 200000U) {
			return HAL_TIMEOUT;
		}
		usb_os_delay_us(1U);
	} while ((USB_GLOBAL->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == USB_OTG_GRSTCTL_RXFFLSH);
	usb_os_delay_us(1U);

	return HAL_OK;
}

/**
  * @brief  Writes a packet into the Tx FIFO associated with the EP/channel
  * @param  src   pointer to source buffer
  * @param  ep_ch_num  endpoint number
  * @param  len  Number of bytes to write
  * @retval Status
  */
USB_TEXT_SECTION
u8 usb_hal_write_packet(u8 *src, u8 ep_ch_num, u16 len)
{
	usb_unaligned_u32_t *p = (usb_unaligned_u32_t *)src;
	u32 count32b, i;
	u32 idx = ep_ch_num;

	DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "usb_hal_write_packet EP%d len=%d\n", ep_ch_num, len);

	count32b = ((u32)len + 3U) / 4U;
	for (i = 0U; i < count32b; i++) {
		USB_DFIFO(idx) = p->data;
		p++;
	}

	return HAL_OK;
}

/**
  * @brief  Read a packet from the Tx FIFO associated with the EP/channel
  * @param  dest  source pointer
  * @param  ep_ch_num  endpoint number
  * @param  len  Number of bytes to read
  * @retval Status
  */
USB_TEXT_SECTION
u8 usb_hal_read_packet(u8 *dest, u8 ep_ch_num, u16 len)
{
	usb_unaligned_u32_t *p = (usb_unaligned_u32_t *)dest;
	u32 i;
	u32 count32b = ((u32)len + 3U) / 4U;
	u32 idx = ep_ch_num;

	for (i = 0U; i < count32b; i++) {
		p->data = USB_DFIFO(idx);
		p++;
	}

	DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "usb_hal_read_packet len=%d\n", len);

	return HAL_OK;
}

#if CONFIG_USB_PHY

/**
  * @brief  Write USB PHY register
  * @param  addr: USB PHY register address
  * @param  data: USB PHY register value
  * @retval Status
  */
USB_TEXT_SECTION
u8 usb_hal_write_phy_register(u8 addr, u8 value)
{
	u8 ret;
	u32 reg;

	reg = HAL_READ32(USB_REG_BASE, USB_ADDON_REG_VND_STS_OUT);
	reg &= (~USB_ADDON_REG_VND_STS_OUT_DATA_MASK);
	reg |= (value << USB_ADDON_REG_VND_STS_OUT_DATA_POS);
	HAL_WRITE32(USB_REG_BASE, USB_ADDON_REG_VND_STS_OUT, reg);

	/* Load low addr */
	ret = usb_hal_load_phy_vendor_control_register(USB_OTG_PHY_LOW_ADDR(addr));
	if (ret == HAL_OK) {
		/* Load high addr */
		ret = usb_hal_load_phy_vendor_control_register(USB_OTG_PHY_HIGH_ADDR(addr));
	}

	return ret;
}

/**
  * @brief  Read USB PHY register
  * @param  addr: USB PHY register address
  * @retval Status
  */
USB_TEXT_SECTION
u8 usb_hal_read_phy_register(u8 addr, u8 *value)
{
	u8 ret;
	u32 reg;
	u8 addr_read;

	if (addr >= 0xE0) {
		addr_read = addr - 0x20;
	} else {
		addr_read = addr;
	}

	/* Load low addr */
	ret = usb_hal_load_phy_vendor_control_register(USB_OTG_PHY_LOW_ADDR(addr_read));
	if (ret == HAL_OK) {
		/* Load high addr */
		ret = usb_hal_load_phy_vendor_control_register(USB_OTG_PHY_HIGH_ADDR(addr_read));
		if (ret == HAL_OK) {
			reg = USB_GLOBAL->GPVNDCTL;
			*value = ((reg & USB_OTG_GPVNDCTL_REGDATA_Msk) >> USB_OTG_GPVNDCTL_REGDATA_Pos) & 0xFF;
		}
	}

	return ret;
}

#if CONFIG_USB_OTG

/**
  * @brief  Enable OTG only for test
  * @retval Status
  */
USB_TEXT_SECTION
u8 usb_hal_enable_otg(void)
{
	u8 ret = HAL_OK;
	u8 reg;

	ret = usb_hal_select_phy_register_page(USB_OTG_PHY_REG_F4_BIT_PAGE1);
	if (ret != HAL_OK) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "Fail to switch phy page 1: %d\n", ret);
		return ret;
	}

	ret = usb_hal_read_phy_register(USB_OTG_PHY_REG_E1, &reg);
	if (ret != HAL_OK) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "Fail to read phy E1/P1: %d\n", ret);
		return ret;
	}

	reg |= USB_OTG_PHY_REG_E1_PAGE1_BIT_EN_OTG;

	ret = usb_hal_write_phy_register(USB_OTG_PHY_REG_E1, reg);
	if (ret != HAL_OK) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "Fail to write phy E1/P1: %d\n", ret);
		return ret;
	}
}

#endif

/**
  * @brief  USB PHY calibration
  * @param  data: USB PHY calibration data
  * @retval Status
  */
USB_TEXT_SECTION
u8 usb_hal_calibrate(u8 mode)
{
	u8 ret = HAL_OK;
	usb_cal_data_t *data;
	u8 old_page = 0xFF;

	data = usb_chip_get_cal_data(mode);
	if (data != NULL) {
		while (data->page != 0xFF) {
			if (data->page != old_page) {
				ret = usb_hal_select_phy_register_page(data->page);
				if (ret != HAL_OK) {
					DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "Fail to switch to page %d: %d\n", data->page, ret);
					break;
				}
				old_page = data->page;
			}
			ret = usb_hal_write_phy_register(data->addr, data->val);
			if (ret != HAL_OK) {
				DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "Fail to write page %d register 0x%02X: %d\n", data->page, data->addr, ret);
				break;
			}
			data++;
		}
	}

	return ret;
}

#endif // CONFIG_USB_PHY

