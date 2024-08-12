/**
  ******************************************************************************
  * @file    usbd_core.c
  * @author  Realsil WLAN5 Team
  * @brief   This file provides the functionalities of USBD core
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

/* Private types -------------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

static void usbd_core_get_descriptor(usb_dev_t *dev, usb_setup_req_t *req);
static void usbd_core_set_address(usb_dev_t *dev, usb_setup_req_t *req);
static void usbd_core_set_config(usb_dev_t *dev, usb_setup_req_t *req);
static void usbd_core_get_config(usb_dev_t *dev, usb_setup_req_t *req);
static void usbd_core_get_status(usb_dev_t *dev, usb_setup_req_t *req);
static void usbd_core_set_feature(usb_dev_t *dev, usb_setup_req_t *req);
static void usbd_core_clear_feature(usb_dev_t *dev, usb_setup_req_t *req);
static u8 usbd_core_ep0_continue_receive_data(usb_dev_t *dev, u8 *buf, u16 len);
static u8 usbd_core_ep0_continue_transmit_data(usb_dev_t *dev, u8 *buf, u16 len);
static void usbd_core_parse_setup_request(usb_setup_req_t *req, u8 *buf);
#if 0
static u8 usbd_core_ep_flush(usb_dev_t *dev, u8 ep_addr);
#endif
static u8 usbd_core_handle_ep_request(usb_dev_t *dev, usb_setup_req_t  *req);
static u8  usbd_core_handle_interface_request(usb_dev_t *dev, usb_setup_req_t  *req);
static u8 usbd_core_handle_device_request(usb_dev_t *dev, usb_setup_req_t  *req);
static u8 usbd_core_set_class_config(usb_dev_t  *dev, u8 config);
static u8 usbd_core_clear_class_config(usb_dev_t  *dev, u8 config);

/* Private variables ---------------------------------------------------------*/

USB_BSS_SECTION
static u8 usbd_cfg;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Handle get descriptor requests
  * @param  dev: USB device instance
  * @param  req: USB request
  * @retval Status
  */
USB_TEXT_SECTION
static void usbd_core_get_descriptor(usb_dev_t *dev, usb_setup_req_t *req)
{
	u16 len;
	u8 *buf;
	u32 dma = ((usbd_pcd_t *)dev->pcd)->config.dma_enable;

	buf = dev->driver->get_descriptor(dev, req, (usb_speed_type_t)dev->dev_speed, &len);
	if (buf == NULL) {
		//usbd_core_ep0_set_stall(dev);
		usbd_core_ep_set_stall(dev, 0x80U);
		return;
	}

	if ((dma == 1) && (!USB_IS_MEM_DMA_ALIGNED(buf))) {
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_ERROR, "Descriptor buffer is not 32 bytes aligned in dma mode!\n");
		usbd_core_ep_set_stall(dev, 0x80U);
		return;
	}

	if ((len != 0U) && (req->wLength != 0U)) {

		len = MIN(len, req->wLength);

		usbd_ep0_transmit(dev, buf, len);
	}

	if (req->wLength == 0U) {
		usbd_core_ep0_transmit_status(dev);
	}
}

/**
  * @brief  Set device address
  * @param  dev: USB device instance
  * @param  req: USB request
  * @retval Status
  */
USB_TEXT_SECTION
static void usbd_core_set_address(usb_dev_t *dev, usb_setup_req_t *req)
{
	u8  dev_addr;

	if ((req->wIndex == 0U) && (req->wLength == 0U) && (req->wValue < 128U)) {
		dev_addr = (u8)(req->wValue) & 0x7FU;

		if (dev->dev_state == USBD_STATE_CONFIGURED) {
			usbd_core_ep0_set_stall(dev);
		} else {
			usbd_pcd_set_address(dev->pcd, dev_addr);
			usbd_core_ep0_transmit_status(dev);

			if (dev_addr != 0U) {
				dev->dev_state = USBD_STATE_ADDRESSED;
			} else {
				dev->dev_state = USBD_STATE_DEFAULT;
			}
		}
	} else {
		usbd_core_ep0_set_stall(dev);
	}
}

/**
  * @brief  Handle set device configuration request
  * @param  dev: USB device instance
  * @param  req: USB request
  * @retval Status
  */
USB_TEXT_SECTION
static void usbd_core_set_config(usb_dev_t *dev, usb_setup_req_t *req)
{
	usbd_cfg = (u8)(req->wValue);

	if (usbd_cfg > USBD_MAX_NUM_CONFIGURATION) {
		usbd_core_ep0_set_stall(dev);
	} else {
		switch (dev->dev_state) {
		case USBD_STATE_ADDRESSED:
			if (usbd_cfg) {
				dev->dev_config = usbd_cfg;
				dev->dev_state = USBD_STATE_CONFIGURED;
				if (usbd_core_set_class_config(dev, usbd_cfg) == HAL_ERR_HW) {
					usbd_core_ep0_set_stall(dev);
					return;
				}
				usbd_core_ep0_transmit_status(dev);
			} else {
				usbd_core_ep0_transmit_status(dev);
			}
			break;

		case USBD_STATE_CONFIGURED:
			if (usbd_cfg == 0U) {
				dev->dev_state = USBD_STATE_ADDRESSED;
				dev->dev_config = usbd_cfg;
				usbd_core_clear_class_config(dev, usbd_cfg);
				usbd_core_ep0_transmit_status(dev);
			} else if (usbd_cfg != dev->dev_config) {
				/* Clear old configuration */
				usbd_core_clear_class_config(dev, dev->dev_config);

				/* set new configuration */
				dev->dev_config = usbd_cfg;
				if (usbd_core_set_class_config(dev, usbd_cfg) == HAL_ERR_HW) {
					usbd_core_ep0_set_stall(dev);
					return;
				}
				usbd_core_ep0_transmit_status(dev);
			} else {
				usbd_core_ep0_transmit_status(dev);
			}
			break;

		default:
			usbd_core_ep0_set_stall(dev);
			usbd_core_clear_class_config(dev, usbd_cfg);
			break;
		}
	}
}

/**
  * @brief  Handle get device configuration request
  * @param  dev: USB device instance
  * @param  req: USB request
  * @retval Status
  */
USB_TEXT_SECTION
static void usbd_core_get_config(usb_dev_t *dev, usb_setup_req_t *req)
{
	if (req->wLength != 1U) {
		usbd_core_ep0_set_stall(dev);
	} else {
		switch (dev->dev_state) {
		case USBD_STATE_DEFAULT:
		case USBD_STATE_ADDRESSED:
			dev->ctrl_buf[0] = 0U;
			usbd_ep0_transmit(dev, dev->ctrl_buf, 1U);
			break;

		case USBD_STATE_CONFIGURED:
			usbd_ep0_transmit(dev, &dev->dev_config, 1U);
			break;

		default:
			usbd_core_ep0_set_stall(dev);
			break;
		}
	}
}

/**
  * @brief  Handle get status request
  * @param  dev: USB device instance
  * @param  req: USB request
  * @retval Status
  */
USB_TEXT_SECTION
static void usbd_core_get_status(usb_dev_t *dev, usb_setup_req_t *req)
{
	u8 status = 0U;

	switch (dev->dev_state) {
	case USBD_STATE_DEFAULT:
	case USBD_STATE_ADDRESSED:
	case USBD_STATE_CONFIGURED:
		if (req->wLength != 0x2U) {
			usbd_core_ep0_set_stall(dev);
			break;
		}

		if (dev->self_powered) {
			status = USB_CONFIG_SELF_POWERED;
		}

		if (dev->remote_wakeup) {
			status |= USB_CONFIG_REMOTE_WAKEUP;
		}

		dev->ctrl_buf[0] = status;
		dev->ctrl_buf[1] = 0U;

		usbd_ep0_transmit(dev, dev->ctrl_buf, 2U);
		break;

	default :
		usbd_core_ep0_set_stall(dev);
		break;
	}
}

/**
  * @brief  Handle set device feature request
  * @param  dev: USB device instance
  * @param  req: USB request
  * @retval Status
  */
USB_TEXT_SECTION
static void usbd_core_set_feature(usb_dev_t *dev, usb_setup_req_t *req)
{
	if (req->wValue == USB_FEATURE_REMOTE_WAKEUP) {
		if (dev->remote_wakeup_en) {
			dev->remote_wakeup = 1U;
		}
	} else if (req->wValue == USB_FEATURE_TEST_MODE) {
		dev->test_mode = (u8)(req->wIndex >> 8) & 0xFF;
	}

	usbd_core_ep0_transmit_status(dev);
}

/**
  * @brief  Handle clear device feature request
  * @param  dev: USB device instance
  * @param  req: USB request
  * @retval Status
  */
USB_TEXT_SECTION
static void usbd_core_clear_feature(usb_dev_t *dev, usb_setup_req_t *req)
{
	switch (dev->dev_state) {
	case USBD_STATE_DEFAULT:
	case USBD_STATE_ADDRESSED:
	case USBD_STATE_CONFIGURED:
		if (req->wValue == USB_FEATURE_REMOTE_WAKEUP) {
			dev->remote_wakeup = 0U;
			usbd_core_ep0_transmit_status(dev);
		}
		break;

	default :
		usbd_core_ep0_set_stall(dev);
		break;
	}
}

/**
  * @brief  Continue receive data on the CTRL pipe
  * @param  dev: USB device instance
  * @param  buf: Data buffer
  * @param  len: Data length
  * @retval Status
  */
USB_TEXT_SECTION
static u8 usbd_core_ep0_continue_receive_data(usb_dev_t *dev, u8 *buf, u16 len)
{
	usbd_ep_receive(dev, 0U, buf, len);

	return HAL_OK;
}

/**
  * @brief  Continue sending data on the CTRL pipe
  * @param  dev: USB device instance
  * @param  buf: Data buffer
  * @param  len: Data length
  * @retval Status
  */
USB_TEXT_SECTION
static u8 usbd_core_ep0_continue_transmit_data(usb_dev_t *dev, u8 *buf, u16 len)
{
	/* Start the next transfer */
	usbd_ep_transmit(dev, 0x00U, buf, len);

	return HAL_OK;
}

/**
  * @brief  Copy buffer into setup structure
  * @param  dev: USB device instance
  * @param  req: USB request
  * @retval None
  */
USB_TEXT_SECTION
static void usbd_core_parse_setup_request(usb_setup_req_t *req, u8 *data)
{
	req->bmRequestType = *(u8 *)(data);
	req->bRequest = *(u8 *)(data + 1);
	req->wValue = (u16)(*(u8 *)(data + 2U)) + ((u16)(*(u8 *)(data + 3U)) << 8U);
	req->wIndex = (u16)(*(u8 *)(data + 4U)) + ((u16)(*(u8 *)(data + 5U)) << 8U);
	req->wLength = (u16)(*(u8 *)(data + 6U)) + ((u16)(*(u8 *)(data + 7U)) << 8U);

}

#if 0
/**
  * @brief  Flushes an endpoint
  * @param  dev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Status
  */
USB_TEXT_SECTION
static u8 usbd_core_ep_flush(usb_dev_t *dev, u8 ep_addr)
{
	usbd_pcd_ep_flush(dev->pcd, ep_addr);
	return HAL_OK;
}
#endif

/**
* @brief  Handle standard usb endpoint requests
* @param  dev: USB device instance
* @param  req: USB request
* @retval Status
*/
USB_TEXT_SECTION
static u8 usbd_core_handle_ep_request(usb_dev_t *dev, usb_setup_req_t  *req)
{
	u8 ep_addr;
	u8 ret = HAL_OK;
	usbd_pcd_t *pcd = dev->pcd;
	ep_addr  = USB_LOW_BYTE(req->wIndex);

	switch (req->bmRequestType & USB_REQ_TYPE_MASK) {

	case USB_REQ_TYPE_CLASS:
	case USB_REQ_TYPE_VENDOR:
		ret = dev->driver->setup(dev, req);
		if (ret != HAL_OK) {
			usbd_core_ep0_set_stall(dev);
		}
		break;

	case USB_REQ_TYPE_STANDARD:

		switch (req->bRequest) {

		case USB_REQ_SET_FEATURE :

			switch (dev->dev_state) {
			case USBD_STATE_ADDRESSED:
				if ((ep_addr != 0x00U) && (ep_addr != 0x80U)) {
					usbd_core_ep_set_stall(dev, ep_addr);
					usbd_core_ep_set_stall(dev, 0x80U);
				} else {
					usbd_core_ep0_set_stall(dev);
				}
				break;

			case USBD_STATE_CONFIGURED:
				if (req->wValue == USB_FEATURE_EP_HALT) {
					if ((ep_addr != 0x00U) && (ep_addr != 0x80U) && (req->wLength == 0x00U)) {
						usbd_core_ep_set_stall(dev, ep_addr);
					}
				}
				usbd_core_ep0_transmit_status(dev);

				break;

			default:
				usbd_core_ep0_set_stall(dev);
				break;
			}
			break;

		case USB_REQ_CLEAR_FEATURE :

			switch (dev->dev_state) {
			case USBD_STATE_ADDRESSED:
				if ((ep_addr != 0x00U) && (ep_addr != 0x80U)) {
					usbd_core_ep_set_stall(dev, ep_addr);
					usbd_core_ep_set_stall(dev, 0x80U);
				} else {
					usbd_core_ep0_set_stall(dev);
				}
				break;

			case USBD_STATE_CONFIGURED:
				if (req->wValue == USB_FEATURE_EP_HALT) {
					if ((ep_addr & EP_ADDR_MSK) != 0x00U) {
						usbd_core_ep_clear_stall(dev, ep_addr);
					}
					usbd_core_ep0_transmit_status(dev);
				}
				break;

			default:
				usbd_core_ep0_set_stall(dev);
				break;
			}
			break;

		case USB_REQ_GET_STATUS:
			switch (dev->dev_state) {
			case USBD_STATE_ADDRESSED:
				if ((ep_addr != 0x00U) && (ep_addr != 0x80U)) {
					usbd_core_ep0_set_stall(dev);
					break;
				}

				dev->ctrl_buf[0] = 0U;
				dev->ctrl_buf[1] = 0U;

				usbd_ep0_transmit(dev, dev->ctrl_buf, 2U);
				break;

			case USBD_STATE_CONFIGURED:
				if ((ep_addr & 0x80U) == 0x80U) {
					if (pcd->in_ep[ep_addr & EP_ADDR_MSK].is_initialized == 0U) {
						usbd_core_ep0_set_stall(dev);
						break;
					}
				} else {
					if (pcd->out_ep[ep_addr & EP_ADDR_MSK].is_initialized == 0U) {
						usbd_core_ep0_set_stall(dev);
						break;
					}
				}

				if ((ep_addr == 0x00U) || (ep_addr == 0x80U)) {
					dev->ctrl_buf[0] = 0U;
				} else if (usbd_core_ep_is_stall(dev, ep_addr)) {
					dev->ctrl_buf[0] = 1U;
				} else {
					dev->ctrl_buf[0] = 0U;
				}
				dev->ctrl_buf[1] = 0U;

				usbd_ep0_transmit(dev, dev->ctrl_buf, 2U);
				break;

			default:
				usbd_core_ep0_set_stall(dev);
				break;
			}
			break;

		default:
			usbd_core_ep0_set_stall(dev);
			break;
		}
		break;

	default:
		usbd_core_ep0_set_stall(dev);
		break;
	}

	return ret;
}

/**
* @brief  Handle standard usb interface requests
* @param  dev: USB device instance
* @param  req: USB request
* @retval Status
*/
USB_TEXT_SECTION
static u8 usbd_core_handle_interface_request(usb_dev_t *dev, usb_setup_req_t  *req)
{
	u8 ret = HAL_OK;

	DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "usbd_core_handle_interface_request: wIndex=0x%04X bmRequestType=0x%02X bRequest=0x%02X wLength=0x%04X\n", req->wIndex,
			   req->bmRequestType, req->bRequest, req->wLength);

	switch (req->bmRequestType & USB_REQ_TYPE_MASK) {
	case USB_REQ_TYPE_CLASS:
	case USB_REQ_TYPE_VENDOR:
	case USB_REQ_TYPE_STANDARD:
		switch (dev->dev_state) {
		case USBD_STATE_DEFAULT:
		case USBD_STATE_ADDRESSED:
		case USBD_STATE_CONFIGURED:
			if (USB_LOW_BYTE(req->wIndex) <= USBD_MAX_NUM_INTERFACES) {
				if (dev->driver->setup != NULL) {
					ret = dev->driver->setup(dev, req);
					if (ret == HAL_OK) {
						if (req->wLength == 0U) {
							usbd_core_ep0_transmit_status(dev);
						}
					} else {
						usbd_core_ep0_set_stall(dev);
					}
				}
			} else {
				usbd_core_ep0_set_stall(dev);
			}
			break;
		default:
			usbd_core_ep0_set_stall(dev);
			break;
		}
		break;

	default:
		usbd_core_ep0_set_stall(dev);
		break;
	}

	return HAL_OK;
}


/**
  * @brief  Handle standard usb device requests
  * @param  dev: USB device instance
  * @param  req: USB request
  * @retval Status
  */
USB_TEXT_SECTION
static u8 usbd_core_handle_device_request(usb_dev_t *dev, usb_setup_req_t  *req)
{
	u8 ret = HAL_OK;

	switch (req->bmRequestType & USB_REQ_TYPE_MASK) {
	case USB_REQ_TYPE_CLASS:
	case USB_REQ_TYPE_VENDOR:
		ret = dev->driver->setup(dev, req);
		if (ret != HAL_OK) {
			usbd_core_ep0_set_stall(dev);
		}
		break;

	case USB_REQ_TYPE_STANDARD:

		switch (req->bRequest) {
		case USB_REQ_GET_DESCRIPTOR:
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "usbd_core_handle_device_request USB_REQ_GET_DESCRIPTOR\n");
			usbd_core_get_descriptor(dev, req);
			break;

		case USB_REQ_SET_ADDRESS:
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "usbd_core_handle_device_request USB_REQ_SET_ADDRESS\n");
			usbd_core_set_address(dev, req);
			break;

		case USB_REQ_SET_CONFIGURATION:
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "usbd_core_handle_device_request USB_REQ_SET_CONFIGURATION\n");
			usbd_core_set_config(dev, req);
			break;

		case USB_REQ_GET_CONFIGURATION:
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "usbd_core_handle_device_request USB_REQ_GET_CONFIGURATION\n");
			usbd_core_get_config(dev, req);
			break;

		case USB_REQ_GET_STATUS:
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "usbd_core_handle_device_request USB_REQ_GET_STATUS\n");
			usbd_core_get_status(dev, req);
			break;


		case USB_REQ_SET_FEATURE:
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "usbd_core_handle_device_request USB_REQ_SET_FEATURE\n");
			usbd_core_set_feature(dev, req);
			break;

		case USB_REQ_CLEAR_FEATURE:
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "usbd_core_handle_device_request USB_REQ_CLEAR_FEATURE\n");
			usbd_core_clear_feature(dev, req);
			break;

		default:
			DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "usbd_core_handle_device_request unexpected request: 0x%02X\n", req->bRequest);
			usbd_core_ep0_set_stall(dev);
			break;
		}
		break;

	default:
		usbd_core_ep0_set_stall(dev);
		break;
	}

	return ret;
}

/**
  * @brief  Configure device and start the interface
  * @param  dev: USB device instance
  * @param  config: configuration index
  * @retval Status
  */
USB_TEXT_SECTION
static u8 usbd_core_set_class_config(usb_dev_t  *dev, u8 config)
{
	u8   ret = HAL_ERR_HW;

	if (dev->driver != NULL) {
		/* Set configuration  and Start the Class*/
		if (dev->driver->set_config(dev, config) == 0U) {
			ret = HAL_OK;
		}
	}

	return ret;
}

/**
  * @brief  Clear current configuration
  * @param  dev: USB device instance
  * @param  config: configuration index
  * @retval Status
  */
USB_TEXT_SECTION
static u8 usbd_core_clear_class_config(usb_dev_t  *dev, u8 config)
{
	/* Clear configuration  and De-initialize the Class process*/
	dev->driver->clear_config(dev, config);
	return HAL_OK;
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Returns the last transfered packet size.
  * @param  dev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Recived Data Size
  */
USB_TEXT_SECTION
u32 usbd_core_get_rx_data_size(usb_dev_t *dev, u8 ep_addr)
{
	return usbd_pcd_ep_get_rx_data_size(dev->pcd, ep_addr);
}

/**
  * @brief  Handle the setup stage
  * @param  dev: USB device instance
  * @param  setup: Setup data buffer
  * @retval Status
  */
USB_TEXT_SECTION
u8 usbd_core_setup_stage(usb_dev_t *dev, u8 *setup)
{
	usb_setup_req_t setup_req;

	u32 dma = ((usbd_pcd_t *)dev->pcd)->config.dma_enable;

	if (dma == 1) {
		DCache_Invalidate((u32) setup, USBD_SETUP_PACKET_BUF_LEN);
	}

	usbd_core_parse_setup_request(&setup_req, setup);

	dev->ep0_state = USBD_EP0_SETUP;

	dev->ep0_data_len = setup_req.wLength;

	switch (setup_req.bmRequestType & 0x1FU) {
	case USB_REQ_RECIPIENT_DEVICE:
		usbd_core_handle_device_request(dev, &setup_req);
		break;

	case USB_REQ_RECIPIENT_INTERFACE:
		usbd_core_handle_interface_request(dev, &setup_req);
		break;

	case USB_REQ_RECIPIENT_ENDPOINT:
		usbd_core_handle_ep_request(dev, &setup_req);
		break;

	default:
		DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "EP0 unexpected request: 0x%08X\n", setup_req.bmRequestType);
		usbd_core_ep_set_stall(dev, (setup_req.bmRequestType & 0x80U));
		break;
	}

	return HAL_OK;
}

/**
  * @brief  Handle data OUT stage
  * @param  dev: USB device instance
  * @param  ep_num: Endpoint index
  * @param  buf: Data buffer
  * @retval Status
  */
USB_TEXT_SECTION
u8 usbd_core_data_out_stage(usb_dev_t *dev, u8 ep_num, u8 *buf)
{
	usbd_pcd_t *pcd = dev->pcd;
	usbd_pcd_ep_t *pep;
	u16 len;
	u32 dma = pcd->config.dma_enable;

	if (ep_num == 0U) {
		pep = &pcd->out_ep[0];
		if (dev->ep0_state == USBD_EP0_DATA_OUT) {
			if (dev->ep0_recv_rem_len > pep->max_packet_len) {
				dev->ep0_recv_rem_len -= pep->max_packet_len;

				usbd_core_ep0_continue_receive_data(dev, buf, (u16)MIN(dev->ep0_recv_rem_len, pep->max_packet_len));
			} else {
				dev->ep0_recv_rem_len = 0;
				if ((dev->driver->ep0_data_out != NULL) &&
					(dev->dev_state == USBD_STATE_CONFIGURED)) {
					if (dma == 1) {
						DCache_Invalidate((u32) pep->dma_addr, pep->xfer_len);
					}
					dev->driver->ep0_data_out(dev);
				}
				usbd_core_ep0_transmit_status(dev);
			}
		} else {
			if (dev->ep0_state == USBD_EP0_STATUS_OUT) {
				/*
				 * STATUS PHASE completed, update ep0_state to idle
				 */
				dev->ep0_state = USBD_EP0_IDLE;
				//usbd_core_ep_set_stall(dev, 0U);  // FIXME
			}
		}
	} else if ((dev->driver->ep_data_out != NULL) &&
			   (dev->dev_state == USBD_STATE_CONFIGURED)) {
		pep = &pcd->out_ep[ep_num];
		len = usbd_core_get_rx_data_size(dev, ep_num);

		if (dma == 1) {
			DCache_Invalidate((u32) pep->dma_addr, pep->xfer_len);
		}
		dev->driver->ep_data_out(dev, ep_num, len);
	} else {
		/* should never be in this condition */
		return HAL_ERR_HW;
	}

	return HAL_OK;
}

/**
  * @brief  Handle data in stage
  * @param  dev: USB device instance
  * @param  ep_num: endpoint index
  * @param  buf: Data buffer
  * @retval Status
  */
USB_TEXT_SECTION
u8 usbd_core_data_in_stage(usb_dev_t *dev, u8 ep_num, u8 *buf, u8 status)
{
	usbd_pcd_t *pcd = dev->pcd;
	usbd_pcd_ep_t *pep;
	u32 dma = pcd->config.dma_enable;

	pep = &pcd->in_ep[ep_num];
	if (status == HAL_OK) {
#ifdef USBD_XFER_ERROR_DETECT_EN
		pep->nptx_err_cnt = 0;
#endif
		if (!(pep->is_ptx)) {
			pcd->nptx_epmis_cnt = 0;
		}

		if (ep_num == 0U) {
			if (dev->ep0_state == USBD_EP0_DATA_IN) {
				if (dev->ep0_xfer_rem_len > pep->max_packet_len) {
					dev->ep0_xfer_rem_len -= pep->max_packet_len;

					usbd_core_ep0_continue_transmit_data(dev, buf, (u16)dev->ep0_xfer_rem_len);

					/* Prepare endpoint for premature end of transfer */
					if (dma != 1) {
						usbd_ep_receive(dev, 0U, NULL, 0U);
					}
				} else {
					dev->ep0_xfer_rem_len = 0;
					/* last packet is MPS multiple, so send ZLP packet */
					if ((dev->ep0_xfer_total_len % pep->max_packet_len == 0U) &&
						(dev->ep0_xfer_total_len >= pep->max_packet_len) &&
						(dev->ep0_xfer_total_len < dev->ep0_data_len)) {
						usbd_core_ep0_continue_transmit_data(dev, NULL, 0U);
						dev->ep0_data_len = 0U;

						/* Prepare endpoint for premature end of transfer */
						if (dma != 1) {
							usbd_ep_receive(dev, 0U, NULL, 0U);
						}
					} else {
						if ((dev->driver->ep0_data_in != NULL) &&
							(dev->dev_state == USBD_STATE_CONFIGURED)) {
							dev->driver->ep0_data_in(dev, HAL_OK);
						}
						//usbd_core_ep_set_stall(dev, 0x80U);  // FIXME
						usbd_core_ep0_receive_status(dev);
					}
				}
			} else {
				if ((dev->ep0_state == USBD_EP0_STATUS_IN) ||
					(dev->ep0_state == USBD_EP0_IDLE)) {
					//usbd_core_ep_set_stall(dev, 0x80U);
				}
			}

			if (dev->test_mode != 0U) {
				usbd_hal_test_mode(dev->test_mode);
				dev->test_mode = 0U;
			}
		} else if ((dev->driver->ep_data_in != NULL) &&
				   (dev->dev_state == USBD_STATE_CONFIGURED)) {
			dev->driver->ep_data_in(dev, ep_num, HAL_OK);
		} else {
			/* should never be in this condition */
			return HAL_ERR_HW;
		}
	} else {
		if (ep_num == 0U) {
			if ((dev->driver->ep0_data_in != NULL) &&
				(dev->dev_state == USBD_STATE_CONFIGURED)) {
				dev->driver->ep0_data_in(dev, status);
			}
		} else {
			if ((dev->driver->ep_data_in != NULL) &&
				(dev->dev_state == USBD_STATE_CONFIGURED)) {
				dev->driver->ep_data_in(dev, ep_num, status);
			}
		}
	}

	return HAL_OK;
}

/**
  * @brief  Handle reset event
  * @param  dev: USB device instance
  * @retval Status
  */
USB_TEXT_SECTION
u8 usbd_core_reset(usb_dev_t  *dev)
{
	/* Open EP0 OUT */
	usbd_ep_init(dev, 0x00U, USB_CH_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);

	/* Open EP0 IN */
	usbd_ep_init(dev, 0x80U, USB_CH_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);

	/* Upon Reset call user call back */
	dev->dev_state = USBD_STATE_DEFAULT;
	dev->ep0_state = USBD_EP0_IDLE;
	dev->dev_config = 0U;
	dev->remote_wakeup = 0U;

	dev->driver->clear_config(dev, dev->dev_config);

	return HAL_OK;
}

/**
  * @brief  Set USB device speed
  * @param  dev: USB device instance
  * @param  speed: Device speed
  * @retval Status
  */
USB_TEXT_SECTION
u8 usbd_core_set_speed(usb_dev_t  *dev, usb_speed_type_t speed)
{
	dev->dev_speed = speed;
	return HAL_OK;
}

/**
  * @brief  Handle suspend event
  * @param  dev: USB device instance
  * @retval Status
  */
USB_TEXT_SECTION
u8 usbd_core_suspend(usb_dev_t  *dev)
{
	dev->dev_old_state =  dev->dev_state;
	dev->dev_state  = USBD_STATE_SUSPENDED;
	if (dev->driver->suspend != NULL) {
		dev->driver->suspend(dev);
	}
	return HAL_OK;
}

/**
  * @brief  Handle resume event
  * @param  dev: USB device instance
  * @retval Status
  */
USB_TEXT_SECTION
u8 usbd_core_resume(usb_dev_t  *dev)
{
	dev->dev_state = dev->dev_old_state;
	if (dev->driver->resume != NULL) {
		dev->driver->resume(dev);
	}
	return HAL_OK;
}

/**
  * @brief  Handle SOF event
  * @param  dev: USB device instance
  * @retval Status
  */
USB_TEXT_SECTION
u8 usbd_core_sof(usb_dev_t  *dev)
{
	if (dev->dev_state == USBD_STATE_CONFIGURED) {
		if (dev->driver->sof != NULL) {
			dev->driver->sof(dev);
		}
	}
	return HAL_OK;
}
/**
  * @brief  Handle EOPF event
  * @param  dev: USB device instance
  * @retval Status
  */
USB_TEXT_SECTION
u8 usbd_core_eopf(usb_dev_t  *dev)
{
	if (dev->dev_state == USBD_STATE_CONFIGURED) {
		usbd_pcd_t *pcd = dev->pcd;
		usbd_pcd_ep_t *ep;
		u8 i;

		for (i = 1U; i < USB_MAX_ENDPOINTS; i++) {
			ep = &(pcd->out_ep[i]);  //out
			if ((USB_CH_EP_TYPE_ISOC == ep->type) && (ep->is_in == 0) && ((USB_OUTEP(ep->num)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA)) {
				if ((USB_DEVICE->DSTS & BIT8) == 0U) { //work in next SOF
					USB_OUTEP(ep->num)->DOEPCTL |= USB_OTG_DOEPCTL_SODDFRM;
				} else {
					USB_OUTEP(ep->num)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
				}
			}

			ep = &(pcd->in_ep[i]); //isoc in, write data to fifo
			if ((1 == ep->data_to_fifo) && (ep->type == USB_CH_EP_TYPE_ISOC) && (ep->is_in == 1) &&
				((USB_INEP(ep->num)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA)) {
				usb_hal_write_packet(ep->xfer_buff, ep->num, (u16)ep->xfer_len);
				ep->data_to_fifo = 0;
			}
		}

	}
	return HAL_OK;
}

/**
  * @brief  Handle device connection event
  * @param  dev: USB device instance
  * @retval Status
  */
USB_TEXT_SECTION
u8 usbd_core_connected(usb_dev_t  *dev)
{
	/* Prevent unused argument compilation warning */
	UNUSED(dev);

	return HAL_OK;
}

/**
  * @brief  Handle device disconnection event
  * @param  dev: USB device instance
  * @retval Status
  */
USB_TEXT_SECTION
u8 usbd_core_disconnected(usb_dev_t  *dev)
{
	/* Free Class Resources */
	dev->dev_state = USBD_STATE_DEFAULT;
	dev->driver->clear_config(dev, dev->dev_config);

	return HAL_OK;
}

/**
  * @brief  Set a stall condition on an endpoint
  * @param  dev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Status
  */
USB_TEXT_SECTION
u8 usbd_core_ep_set_stall(usb_dev_t *dev, u8 ep_addr)
{
	DBG_PRINTF(MODULE_USB_OTG, LEVEL_TRACE, "usbd_core_ep_set_stall %d\n", ep_addr);
	usbd_pcd_ep_set_stall(dev->pcd, ep_addr);
	return HAL_OK;
}

/**
  * @brief  Clears a stall condition on an endpoint
  * @param  dev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Status
  */
USB_TEXT_SECTION
u8 usbd_core_ep_clear_stall(usb_dev_t *dev, u8 ep_addr)
{
	usbd_pcd_ep_clear_stall(dev->pcd, ep_addr);
	return HAL_OK;
}

/**
  * @brief  Returns Stall condition.
  * @param  dev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Stall (1: yes, 0: No)
  */
USB_TEXT_SECTION
u8 usbd_core_ep_is_stall(usb_dev_t *dev, u8 ep_addr)
{
	usbd_pcd_t *pcd = dev->pcd;

	if ((ep_addr & 0x80) == 0x80) {
		return pcd->in_ep[ep_addr & 0xF].is_stall;
	} else {
		return pcd->out_ep[ep_addr & 0xF].is_stall;
	}
}

/**
  * @brief  Handle USB low level Error
  * @param  dev: USB device instance
  * @param  req: USB request
  * @retval None
  */
USB_TEXT_SECTION
u8 usbd_core_ep0_set_stall(usb_dev_t *dev)
{
	usbd_core_ep_set_stall(dev, 0x80U);
	usbd_core_ep_set_stall(dev, 0U);

	return HAL_OK;
}

/**
  * @brief  Send zero length packet to the CTRL pipe
  * @param  dev: USB device instance
  * @retval Status
  */
USB_TEXT_SECTION
u8 usbd_core_ep0_transmit_status(usb_dev_t *dev)
{
	/* Set EP0 State */
	dev->ep0_state = USBD_EP0_STATUS_IN;

	/* Start the transfer */
	usbd_ep_transmit(dev, 0x00U, NULL, 0U);

	return HAL_OK;
}

/**
  * @brief  Receive zero length packet from the CTRL pipe
  * @param  dev: USB device instance
  * @retval Status
  */
USB_TEXT_SECTION
u8 usbd_core_ep0_receive_status(usb_dev_t *dev)
{
	/* Set EP0 State */
	dev->ep0_state = USBD_EP0_STATUS_OUT;

	/* Start the transfer */
	usbd_ep_receive(dev, 0U, NULL, 0U);

	return HAL_OK;
}


