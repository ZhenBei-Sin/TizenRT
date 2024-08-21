/****************************************************************************
 *
 * Copyright 2024 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <tinyara/arch.h>
#include <tinyara/kmalloc.h>
//zhenbei: causing the redefine
// usb_ch9.h & 
//#include <tinyara/usb/usb.h>
//#include <tinyara/usb/usbdev.h>
#include <tinyara/usb/usbdev_trace.h>

#include <arch/irq.h>
#include <tinyara/semaphore.h>
#if defined(CONFIG_AMEBASMART_USBDEVICE)
#include "usbd.h"
#include "usb_os.h"
#include "usbd_hal.h"
#include "usbd_pcd.h"
#include "usbd_core.h"
#include "usbd_cdc_acm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

//zhenbei: need to put in defconfig
#define CONFIG_AMEBASMART_USBD_CDC_ACM_ASYNC_XFER	0
#define CONFIG_AMEBASMART_CDC_ACM_NOTIFY		0
#define CONFIG_AMEBASMART_USBD_CDC_ACM_HOTPLUG		0

// Asynchronous transfer size
#define CONFIG_CDC_ACM_ASYNC_BUF_SIZE			2048U

// Do not change the settings unless indeed necessary
#define CONFIG_CDC_ACM_BULK_IN_XFER_SIZE		2048U
#define CONFIG_CDC_ACM_BULK_OUT_XFER_SIZE		2048U

#if CONFIG_AMEBASMART_USBD_CDC_ACM_ASYNC_XFER
static u8 cdc_acm_async_xfer_buf[CONFIG_CDC_ACM_ASYNC_BUF_SIZE] __attribute_((aligned(CACHE_LINE_SIZE)));
static u16 cdc_acm_async_xfer_buf_pos = 0;
static volatile int cdc_acm_async_xfer_busy = 0;
static _sema cdc_acm_async_xfer_sema;
#endif

#if CONFIG_AMEBASMART_USBD_CDC_ACM_HOTPLUG
static u8 cdc_acm_attach_status;
static _sema cdc_acm_attach_status_changed_sema;
#endif

#define CONFIG_TEST_TRANSMIT				0

#endif /* CONFIG_AMEBASMART_USBDEVICE */

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* The various states of a control pipe */
/* zhenbei: refer to the usbd_core.h */
/*  EP0 State */
#define USBD_EP0_IDLE                                  0x00U    /* No request in progress */
#define USBD_EP0_SETUP                                 0x01U    /* Setup packet received, preparing for OUT transfer */
#define USBD_EP0_DATA_IN                               0x02U    /* Device ready to send data to host (IN transfer) */
#define USBD_EP0_DATA_OUT                              0x03U    /* Host sending data to device (OUT transfer) */
#define USBD_EP0_STATUS_IN                             0x04U    /* Waiting for host to send status handshake (IN transfer) */
#define USBD_EP0_STATUS_OUT                            0x05U    /* Waiting for device to send status handshake (OUT transfer) */
#define USBD_EP0_STALL                                 0x06U    /* Endpoint stalled due to error or unsupported request */


/* zhenbei: refer to the usbd.h  USB device: usb_dev_t */
struct amebasmart_usbdev_s {
	/* Common device fields.  This must be the first thing defined in the
	 * structure so that it is possible to simply cast from struct usbdev_s
	 * to struct amebasmart_usbdev_s.
	 */

	struct _usbd_class_driver_t *driver;	/* Class driver */
	u32 ep0_xfer_total_len;			/* The total data length to transfer */
	u32 ep0_xfer_rem_len;			/* The remain data length to transfer */
	u32 ep0_recv_rem_len;			/* The remain data length to receive */
	u8 *ctrl_buf;				/* Buffer for control transfer */
	void *pcd;				/* PCD handle */
	u16 ep0_data_len;			/* EP0 data length */
	u8 ep0_state;				/* EP0 state */
	u8 dev_config;				/* Device config index */
	u8 dev_speed;				/* Device speed, usb_speed_type_t */
	u8 dev_state;				/* Device state, usbd_state_t */
	u8 dev_old_state;			/* Device old state, usbd_state_t */
	u8 dev_attach_status;			/* Device attach status, usbd_attach_status_t */
	u8 test_mode;				/* Test mode */
	u8 self_powered : 1;			/* Self powered or not, 0-bus powered, 1-self powered */
	u8 remote_wakeup_en : 1;		/* Remote wakeup enable or not, 0-disabled, 1-enabled */
	u8 remote_wakeup : 1;			/* Remote wakeup */
	sem_t txsem;
};


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

//zhenbei: can add the dump_register API & need add the config
//may refer the usb_hal.c usb_hal_dump_registers
#if defined(CONFIG_DEBUG_USBDEVICE)
static void amebasmart_usb_hal_dump_registers(void);
#endif

/* Low-Level Helpers ********************************************************/

/* Suspend/Resume Helpers ***************************************************/

/* Request Helpers **********************************************************/

/* Interrupt level processing ***********************************************/

/* Endpoint helpers *********************************************************/

/* Endpoint operations ******************************************************/

/* USB device controller operations *****************************************/

/* Initialization/Reset *****************************************************/

/*usbd related APIs*/
static int amebasmart_usbd_init(struct amebasmart_usbdev_s *priv, usbd_config_t *cfg);
static int amebasmart_usbd_cdc_acm_init(struct amebasmart_usbdev_s *priv, usbd_cdc_acm_cb_t *cb);

/*CDC ACM related*/
static int amebasmart_up_setup(struct amebasmart_usbdev_s *priv);

static int amebasmart_cdc_acm_cb_init(struct amebasmart_usbdev_s *priv);
static int amebasmart_cdc_acm_cb_deinit(struct amebasmart_usbdev_s *priv);
static int amebasmart_cdc_acm_cb_setup(struct amebasmart_usbdev_s *priv, usb_setup_req_t *req, u8 *buf);
static int amebasmart_cdc_acm_cb_received(struct amebasmart_usbdev_s *priv, u8 *buf, u16 len);
static void amebasmart_cdc_acm_cb_status_changed(struct amebasmart_usbdev_s *priv, u8 status);
static int amebasmart_usbd_cdc_acm_init(struct amebasmart_usbdev_s *priv, usbd_cdc_acm_cb_t *cb);
static amebasmart_usbd_cdc_acm_deinit(struct amebasmart_usbdev_s *priv);
static void amebasmart_cdc_acm_cb_transmitted(struct amebasmart_usbdev_s *priv, u8 status);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static usbd_cdc_acm_cb_t amebasmart_cdc_acm_cb = {
	.init 		= amebasmart_cdc_acm_cb_init,
	.deinit 	= amebasmart_cdc_acm_cb_deinit,
	.setup 		= amebasmart_cdc_acm_cb_setup,
	.received 	= amebasmart_cdc_acm_cb_received,
	.transmitted = amebasmart_cdc_acm_cb_transmitted,
	.status_changed = amebasmart_cdc_acm_cb_status_changed
};

static struct amebasmart_usbdev_s g_usbdev;

/*CDC ACM related*/
/* zhenbei: This priority will affect when test USB print help menu not display
 (Tested 200 or 9 not able to print help menu) */
#define CONFIG_AMEBASMART_CDC_ACM_ISR_THREAD_PRIORITY 25
static usbd_config_t amebasmart_cdc_acm_cfg = {
	.speed = USB_SPEED_FULL,
	.dma_enable   = 1U,
	.isr_priority = CONFIG_AMEBASMART_CDC_ACM_ISR_THREAD_PRIORITY,
	.intr_use_ptx_fifo  = 0U,
	.nptx_max_epmis_cnt = 10U,
	.ext_intr_en        = USBD_EPMIS_INTR,
	.nptx_max_err_cnt   = {0U, 0U, 0U, 2000U, },
};

static usbd_cdc_acm_line_coding_t amebasmart_cdc_acm_line_coding;

static uint16_t cdc_acm_ctrl_line_state;

/**
  * @brief  Initializes the CDC media layer
  * @param  None
  * @retval Status
  */
static int amebasmart_cdc_acm_cb_init(struct amebasmart_usbdev_s *priv) {
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);

	usbd_cdc_acm_line_coding_t *lc = &amebasmart_cdc_acm_line_coding;

	lc->bitrate = 1500000;
	lc->format = 0x00;
	lc->parity_type = 0x00;
	lc->data_type = 0x08;

#if CONFIG_AMEBASMART_USBD_CDC_ACM_ASYNC_XFER
	cdc_acm_async_xfer_buf_pos = 0;
	cdc_acm_async_xfer_busy = 0;
#endif
	return 0;
}

/**
  * @brief  DeInitializes the CDC media layer
  * @param  None
  * @retval Status
  */
static int amebasmart_cdc_acm_cb_deinit(struct amebasmart_usbdev_s *priv) {

	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);

#if CONFIG_AMEBASMART_USBD_CDC_ACM_ASYNC_XFER
	cdc_acm_async_xfer_buf_pos = 0;
	cdc_acm_async_xfer_busy = 0;
#endif
	return HAL_OK;
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface through this function.
  * @param  Buf: RX buffer
  * @param  Len: RX data length (in bytes)
  * @retval Status
  */
static int amebasmart_cdc_acm_cb_received(struct amebasmart_usbdev_s *priv, u8 *buf, u16 len) {
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);

#if CONFIG_TEST_TRANSMIT
	int res = 0;
	int length = 26;
	uint8_t buffer[length];
	for (int i = 0; i < length; i ++){
		buffer[i] = 0x61 + i;
	}
	res = usbd_cdc_acm_transmit(buffer, length);
	dbg("\n[CDC] CONFIG_TEST_TRANSMIT res=%d length=%d\n", res, length);
	dbg("\n[CDC] Transmit Success, data0=0x%02x,len=%d bytes\n", buffer[0], length);
	return HAL_OK;
#endif

#if CONFIG_AMEBASMART_USBD_CDC_ACM_ASYNC_XFER
	u8 ret = HAL_OK;
	if (0 == cdc_acm_async_xfer_busy) {
		if ((cdc_acm_async_xfer_buf_pos + len) > CONFIG_CDC_ACM_ASYNC_BUF_SIZE) {
			len = CONFIG_CDC_ACM_ASYNC_BUF_SIZE - cdc_acm_async_xfer_buf_pos;  // extra data discarded
		}

		rtw_memcpy((void *)((u32)cdc_acm_async_xfer_buf + cdc_acm_async_xfer_buf_pos), buf, len);
		cdc_acm_async_xfer_buf_pos += len;
		if (cdc_acm_async_xfer_buf_pos >= CONFIG_CDC_ACM_ASYNC_BUF_SIZE) {
			cdc_acm_async_xfer_buf_pos = 0;
			rtw_up_sema(&cdc_acm_async_xfer_sema);
		}
	} else {
		printf("\n[CDC] Busy, discarded %d bytes\n", len);
		ret = HAL_BUSY;
	}

	return ret;
#else
	int ret = -1;
	return HAL_OK;
#endif
}

static void amebasmart_cdc_acm_cb_transmitted(struct amebasmart_usbdev_s *priv, u8 status)
{
	(void)sem_post(&g_usbdev.txsem);
}
/**
  * @brief  Handle the CDC class control requests
  * @param  cmd: Command code
  * @param  buf: Buffer containing command data (request parameters)
  * @param  len: Number of data to be sent (in bytes)
  * @retval Status
  */
static int amebasmart_cdc_acm_cb_setup(struct amebasmart_usbdev_s *priv, usb_setup_req_t *req, u8 *buf)
{
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);

	usbd_cdc_acm_line_coding_t *lc = &amebasmart_cdc_acm_line_coding;

	switch (req->bRequest) {
	case CDC_SEND_ENCAPSULATED_COMMAND:
		/* Do nothing */
		lldbg("%d\n",__LINE__);
		break;

	case CDC_GET_ENCAPSULATED_RESPONSE:
		/* Do nothing */
		lldbg("%d\n",__LINE__);
		break;

	case CDC_SET_COMM_FEATURE:
		/* Do nothing */
		lldbg("%d\n",__LINE__);
		break;

	case CDC_GET_COMM_FEATURE:
		/* Do nothing */
		lldbg("%d\n",__LINE__);
		break;

	case CDC_CLEAR_COMM_FEATURE:
		/* Do nothing */
		lldbg("%d\n",__LINE__);
		break;

	case CDC_SET_LINE_CODING:
		if (req->wLength == CDC_ACM_LINE_CODING_SIZE) {
			lc->bitrate = (u32)(buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24));
			lc->format = buf[4];
			lc->parity_type = buf[5];
			lc->data_type = buf[6];
		}
		lldbg("%d bitrate = %d\n",__LINE__,lc->bitrate);

		break;

	case CDC_GET_LINE_CODING:
		buf[0] = (u8)(lc->bitrate & 0xFF);
		buf[1] = (u8)((lc->bitrate >> 8) & 0xFF);
		buf[2] = (u8)((lc->bitrate >> 16) & 0xFF);
		buf[3] = (u8)((lc->bitrate >> 24) & 0xFF);
		buf[4] = lc->format;
		buf[5] = lc->parity_type;
		buf[6] = lc->data_type;
		lldbg("%d bitrate = %d\n",__LINE__,lc->bitrate);
		break;

	case CDC_SET_CONTROL_LINE_STATE:
		/*
		wValue:	wValue, Control Signal Bitmap
				D2-15:	Reserved, 0
				D1:	RTS, 0 - Deactivate, 1 - Activate
				D0:	DTR, 0 - Not Present, 1 - Present
		*/
		cdc_acm_ctrl_line_state = req->wValue;
		if (cdc_acm_ctrl_line_state & 0x01) {
			lldbg("\n[CDC] VCOM port activated\n");
#if CONFIG_AMEBASMART_CDC_ACM_NOTIFY
			usbd_cdc_acm_notify_serial_state(CDC_ACM_CTRL_DSR | CDC_ACM_CTRL_DCD);
#endif
		}
		break;

	case CDC_SEND_BREAK:
		/* Do nothing */
		break;

	default:
		break;
	}

	return HAL_OK;
}

static void amebasmart_cdc_acm_cb_status_changed(struct amebasmart_usbdev_s *priv, uint8_t status) {
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);

#if CONFIG_AMEBASMART_USBD_CDC_ACM_HOTPLUG
	cdc_acm_attach_status = status;
	rtw_up_sema(&cdc_acm_attach_status_changed_sema);
#endif
}


static int amebasmart_usbd_cdc_acm_init(struct amebasmart_usbdev_s *priv, usbd_cdc_acm_cb_t *cb) {
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	int ret = -1;

	ret = usbd_cdc_acm_init(CONFIG_CDC_ACM_BULK_OUT_XFER_SIZE, CONFIG_CDC_ACM_BULK_IN_XFER_SIZE, &amebasmart_cdc_acm_cb);
	if (ret != 0) {
		dbg("USB ACM init failed\n");
		amebasmart_usbd_cdc_acm_deinit(dev);
	}
	return ret;
}

static amebasmart_usbd_cdc_acm_deinit(struct amebasmart_usbdev_s *priv)
{
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);

	usbd_cdc_acm_deinit();
}

static int amebasmart_usbd_init(struct amebasmart_usbdev_s *priv, usbd_config_t *cfg) {
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;

	int ret = -1;
	ret = usbd_init(cfg);

	return ret;
};

/****************************************************************************
 * Name: amebasmart_up_usbuninitialize
 * Description: Initialize the USB driver
 * Input Parameters: None
 * Returned Value: None
 ****************************************************************************/
int amebasmart_up_usbinitialize(struct amebasmart_usbdev_s *priv)
{
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);
	
	int ret = -1;

	ret = amebasmart_usbd_init(dev, &amebasmart_cdc_acm_cfg);
	if (ret != 0) {
		dbg("usbd init fail\n");
		return ret;
	}

	ret = amebasmart_usbd_cdc_acm_init(dev, &amebasmart_cdc_acm_cb);
	if (ret != 0) {
		dbg("usbd acm init fail\n");
		return ret;
	}
	sem_init(&g_usbdev.txsem, 0, 0);
	return ret;
}

void usb_initialize(void)
{
	struct amebasmart_usbdev_s *priv = NULL;
	priv = &g_usbdev;

	int ret = -1;

	ret = amebasmart_up_usbinitialize(priv);
	if (ret != 0) {
		dbg("amebasmart usb init fail\n");
	}
}

int usb_printf(uint8_t *buf, u16 len)
{
	int new_len = 0;
	int ret = -1;
	for(int i = 0; i < len; i++){
		if(buf[i] == '\n'){
			new_len++;
		}
	}
	len += new_len;
	uint8_t *transfer = (uint8_t *)rtw_zmalloc(len);
	if (transfer == NULL){
		lldbg("malloc fail\n");
		return ret;
	}
	int j = 0;
	for (int i = 0; i < len; i++) {
		if(buf[i] == '\n') {
			transfer[j++] = '\r'; 
		}
		transfer[j++] = buf[i];
	}
	transfer[j] = '\0';

	ret = usbd_cdc_acm_transmit(transfer,len);
	while (sem_wait(&g_usbdev.txsem) != 0) {
		/* The only case that an error should occur here is if the wait was awakened
		 * by a signal.
		 */
		ASSERT(errno == EINTR);
	}
	rtw_mfree(transfer, 0);
	return ret;
}

