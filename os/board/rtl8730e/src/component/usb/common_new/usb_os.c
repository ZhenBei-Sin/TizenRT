/**
  ******************************************************************************
  * @file    usb_ch9.h
  * @author  Realsil WLAN5 Team
  * @brief   This file provides general defines for USB SPEC CH9
  ******************************************************************************
  * @attention
  *
  * This module is a confidential and proprietary property of RealTek and
  * possession or use of this module requires written permission of RealTek.
  *
  * Copyright(c) 2021, Realtek Semiconductor Corporation. All rights reserved.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "ameba_soc.h"
#include "usb_os.h"
#include "usb_hal.h"
#include "osdep_service.h"

/* Private defines -----------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Exported defines ----------------------------------------------------------*/

/**
  * @brief  memory set
  * @param  buf: buffer
  * @param  val: value
  * @param  size: size in byte
  * @retval void
  */
USB_TEXT_SECTION
void usb_os_memset(void *buf, u8 val, u32 size)
{
	rtw_memset(buf, val, size);
}

/**
  * @brief  memory copy
  * @param  dst: destination buffer
  * @param  src: source buffer
  * @param  size: size in byte
  * @retval void
  */
USB_TEXT_SECTION
void usb_os_memcpy(void *dst, const void *src, u32 size)
{
	rtw_memcpy(dst, src, size);
}

/**
  * @brief  Delay ms
  * @param  ms: time in ms
  * @retval void
  */
USB_TEXT_SECTION
void usb_os_delay_ms(u32 ms)
{
#ifdef CONFIG_USB_OS
	rtw_msleep_os(ms);
	//rtw_mdelay_os(ms);
	//vtaskdelay
#else
	DelayMs(ms);
#endif
}

/**
  * @brief  Delay us
  * @param  us: time in us
  * @retval void
  */
USB_TEXT_SECTION
void usb_os_delay_us(u32 us)
{
#ifdef CONFIG_USB_OS
	rtw_usleep_os(us);
	//rtw_udelay_os(us);
	//vtaskdelay
#else
	DelayUs(us);
#endif
}

#ifdef CONFIG_USB_OS

/**
  * @brief  Alloc spinloc
  * @param  void
  * @retval Spinloc pointer
  */
USB_TEXT_SECTION
usb_spinlock_t *usb_os_spinlock_alloc(void)
{
	usb_spinlock_t *lock = NULL;
	lock = (usb_spinlock_t *)rtw_zmalloc(sizeof(usb_spinlock_t));
	if (lock != NULL) {
		rtw_spinlock_init((_lock *)lock);
	}
	return lock;
}

/**
  * @brief  Free spinloc
  * @param  lock: spinloc pointer
  * @retval void
  */
USB_TEXT_SECTION
void usb_os_spinlock_free(usb_spinlock_t *lock)
{
	if (lock != NULL) {
		rtw_spinlock_free((_lock *)lock);
		rtw_free((void *)lock);
	}
}

/**
  * @brief  Get spinloc
  * @param  lock: spinloc pointer
  * @retval void
  */
USB_TEXT_SECTION
void usb_os_spinlock(usb_spinlock_t *lock)
{
	rtw_spin_lock((_lock *)lock);
}

/**
  * @brief  Put spinloc
  * @param  lock: spinloc pointer
  * @retval void
  */
USB_TEXT_SECTION
void usb_os_spinunlock(usb_spinlock_t *lock)
{
	rtw_spin_unlock((_lock *)lock);
}

/**
  * @brief  Disable USB interrupt and get spinloc
  * @param  lock: spinloc pointer
  * @retval void
  */
USB_TEXT_SECTION
void usb_os_spinlock_safe(usb_spinlock_t *lock)
{
	usb_hal_disable_interrupt();
	usb_os_spinlock(lock);
}

/**
  * @brief  Put spinlock and enable USB interrupt
  * @param  lock: spinloc pointer
  * @retval void
  */
USB_TEXT_SECTION
void usb_os_spinunlock_safe(usb_spinlock_t *lock)
{
	usb_os_spinunlock(lock);
	usb_hal_enable_interrupt();
}

#endif

