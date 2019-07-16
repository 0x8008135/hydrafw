/*
HydraBus/HydraNFC - Copyright (C) 2014-2015 Benjamin VERNOUX

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#include "common.h"
#include "hydrabus_mode_usb.h"
#include "bsp_usb.h"
#include <string.h>
#include "bsp_usb_conf.h"



#include "stm32f405xx.h"
#include "stm32f4xx_hal.h"
#include "bsp_usb.h"

#define MIN(a, b)  (((a) < (b)) ? (a) : (b))

USB_OTG_GlobalTypeDef* USBx;
USB_OTG_DeviceTypeDef* USBx_dev = (USB_OTG_DeviceTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE);
#define USBx_DEVICE     ((USB_OTG_DeviceTypeDef *)((uint32_t)USBx + USB_OTG_DEVICE_BASE))
#define USBx_INEP(i)    ((USB_OTG_INEndpointTypeDef *)((uint32_t)USBx + USB_OTG_IN_ENDPOINT_BASE + ((i) * USB_OTG_EP_REG_SIZE)))
#define USBx_OUTEP(i)   ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)USBx + USB_OTG_OUT_ENDPOINT_BASE + ((i) * USB_OTG_EP_REG_SIZE)))
#define USBx_DFIFO(i)   *(__IO uint32_t *)((uint32_t)USBx + USB_OTG_FIFO_BASE + ((i) * USB_OTG_FIFO_SIZE))
#define USBx_PCGCCTL    *(__IO uint32_t *)((uint32_t)USBx + USB_OTG_PCGCCTL_BASE)

#define  USB_DESC_TYPE_DEVICE                           0x01
#define  USB_DESC_TYPE_CONFIGURATION                    0x02
#define  USB_DESC_TYPE_STRING                           0x03
#define  USB_DESC_TYPE_INTERFACE                        0x04
#define  USB_DESC_TYPE_ENDPOINT                         0x05
#define  USB_DESC_TYPE_DEVICE_QUALIFIER                 0x06
#define  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION        0x07
#define  USB_DESC_TYPE_BINARY_OBJECT_STORE              0x0f

// offsets for configuration strings
#define  STRING_OFFSET_LANGID                           0x00
#define  STRING_OFFSET_IMANUFACTURER                    0x01
#define  STRING_OFFSET_IPRODUCT                         0x02
#define  STRING_OFFSET_ISERIAL                          0x03
#define  STRING_OFFSET_ICONFIGURATION                   0x04
#define  STRING_OFFSET_IINTERFACE                       0x05
#define  UART_BRIDGE_BUFF_SIZE                          0x40

USB_OTG_EPTypeDef IN_ep[16]; 	//in endpoints
USB_OTG_EPTypeDef OUT_ep[16]; 	//out endpoints


#define STRING_DESCRIPTOR_HEADER(size)\
  (((((size) * 2) + 2) & 0xFF) | 0x0300)

uint8_t device_desc[] = {
  0x12,       		// bLength
  0x01, 	  		// bDescriptorType: 0x01 (DEVICE)
  0x10, 0x01, 		// bcdUSB
  0x02, 	  		// bDeviceClass: CDC Control (0x02)
  0x00, 	  		// bSubclass
  0x00, 	  		// bDeviceProtocol
  0x40, 	  		// bMaxPacketSize0
  0x50, 0x1d, 		// idVendor
  0xa7, 0x60, 		// idProduct
  0x00, 0x02, 		// bcdDevice (0x200)
  0x01,		  		// idManufacturer
  0x02,       		// iProduct
  0x03,  	  		// iSerialNumber
  0x01,	  	  		// bNumConfigurations
};

uint8_t configuration_desc[] = {
  0x09, 0x02,
  0x43, 0x00,
  0x02, 0x01,
  0x00, 0xC0,
  0x32, 0x09,
  0x04, 0x00,
  0x00, 0x01,
  0x02, 0x02,
  0x01, 0x00,
  0x05, 0x24,
  0x00, 0x10,
  0x01, 0x05,
  0x24, 0x01,
  0x00, 0x01,
  0x04, 0x24,
  0x02, 0x02,
  0x05, 0x24,
  0x06, 0x00,
  0x01, 0x07,
  0x05, 0x82,
  0x03, 0x08,
  0x00, 0xFF,
  0x09, 0x04,
  0x01, 0x00,
  0x02, 0x0A,
  0x00, 0x00,
  0x00, 0x07,
  0x05, 0x01,
  0x02, 0x40,
  0x00, 0x00,
  0x07, 0x05,
  0x81, 0x02,
  0x40, 0x00,
  0x00
};

uint8_t device_qualifier[] = {
  0x0a, 0x01, 		//Length, Type
  0x10, 0x02, 		// bcdUSB max version of USB supported (2.1)
  0xFF, 0xFF, 		//
  0xFF, 0x40, 		// bDeviceClass, bDeviceSubClass, bDeviceProtocol, bMaxPacketSize0
  0x01, 0x00 		// bNumConfigurations, bReserved
};

uint16_t string_language_desc[] = {
  STRING_DESCRIPTOR_HEADER(1),
  0x0409 // american english
};

uint16_t string_manufacturer_desc[] = {
  STRING_DESCRIPTOR_HEADER(14),
  'O', 'p', 'e', 'n', 'm', 'o', 'k', 'o', ',', ' ', 'I', 'n', 'c', '.'
};

uint16_t string_product_desc[] = {
  STRING_DESCRIPTOR_HEADER(24),
  'H', 'y', 'd', 'r', 'a', 'B', 'u', 's', ' ', '1', '.', '0', ' ', 'C', 'O', 'M', ' ', 'P',  'o', 'r', 't', '2'
};

// default serial number when we're not a panda
uint16_t string_serial_desc[] = {
  STRING_DESCRIPTOR_HEADER(4),
  'n', 'o', 'n', 'e'
};

// a string containing the default configuration index
uint16_t string_configuration_desc[] = {
  STRING_DESCRIPTOR_HEADER(2),
  '0', '1' // "01"
};


//Setup pkt
typedef union {
  uint16_t w;
  struct BW {
    uint8_t msb;
    uint8_t lsb;
  }
  bw;
} uint16_t_uint8_t;

typedef union _USB_Setup {
  uint32_t d8[2];
  struct _SetupPkt_Struc
  {
    uint8_t           	bmRequestType;
    uint8_t           	bRequest;
    uint16_t_uint8_t	wValue;
    uint16_t_uint8_t	wIndex;
    uint16_t_uint8_t	wLength;
  } b;
}
USB_Setup_TypeDef;

USB_Setup_TypeDef setup;

//send buffer tracking
uint8_t* tx_data = NULL;
uint16_t tx_len = 0;

#define NB_USB (BSP_DEV_USB_END)

//static USBD_HandleTypeDef usbd_handle[NB_USB];
//static mode_config_proto_t* usbd_mode_conf[NB_USB];

/**
  * @brief  USBx error treatment function.
  * @param  dev_num: USB dev num
  * @retval None
  */
static void usb_error(bsp_dev_usb_t dev_num)
{
    (void)dev_num;
	if(bsp_usb_deinit(dev_num) == BSP_OK) {
		/* Re-Initialize the USB communication bus */
		//bsp_usb_init(dev_num, usb_mode_conf[dev_num]);
	}
}

/**
  * @brief  Init USB device.
  * @param  dev_num: USB dev num.
  * @param  mode_conf: Mode config proto.
  * @retval status: status of the init.
  */

bsp_status_t bsp_usb_init(bsp_dev_usb_t dev_num, mode_config_proto_t* mode_conf)
{
	bsp_status_t status;
    USB_OTG_CfgTypeDef cfg; 		//hpcd->Init

    uint8_t i;

    GPIO_InitTypeDef GPIO_InitStruct;
    //Start: USBD_LL_Init
    /* GPIOB clock enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /**USB_OTG_HS GPIO Configuration
    PB14     ------> USB_OTG_HS_DM
    PB15     ------> USB_OTG_HS_DP
    */

    GPIO_InitStruct.Pin = BSP_USB2_OTG_HS_DM|BSP_USB2_OTG_HS_DP;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = BSP_USB2_OTG_HS_SPEED;
    GPIO_InitStruct.Alternate = BSP_USB2_OTG_HS_AF;
    HAL_GPIO_Init(BSP_USB2_OTG_HS_PORT, &GPIO_InitStruct);

    /* Configure VBUS Pin */
    GPIO_InitStruct.Pin = BSP_USB2_OTG_HS_VBUS;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BSP_USB2_OTG_HS_PORT, &GPIO_InitStruct);

    /* Configure ID pin */
    GPIO_InitStruct.Pin = BSP_USB2_OTG_HS_ID;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Alternate = BSP_USB2_OTG_HS_AF;
    HAL_GPIO_Init(BSP_USB2_OTG_HS_PORT, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_USB_OTG_HS_CLK_ENABLE();

    USBx = USB_OTG_HS;

    cfg.dev_endpoints = 1;
    cfg.dma_enable = 0;
    cfg.ep0_mps = DEP0CTL_MPS_64;
    cfg.low_power_enable = 0;
    cfg.lpm_enable = 0;
    cfg.phy_itface = USB_OTG_EMBEDDED_PHY;
    cfg.Sof_enable = 0;
    cfg.speed = PCD_SPEED_FULL;
    cfg.use_dedicated_ep1 = 0;
    cfg.use_external_vbus = 0;
	cfg.vbus_sensing_enable = 0;

	USBx->GRXFSIZ = 0x40;

	USBx->DIEPTXF0_HNPTXFSIZ = (0x40 << 16) | 0x40;

	USBx->DIEPTXF[0] = (0x40 << 16) | 0x80;

	USBx_OUTEP(0)->DOEPTSIZ = USB_OTG_DOEPTSIZ_STUPCNT | (USB_OTG_DOEPTSIZ_PKTCNT & (1 << 19)) | (3 * 8);
	//End: USBD_LL_Init

	//start: USBD_Init
	USB_CoreInit(USBx, cfg);
	USB_SetCurrentMode(USBx, USB_DEVICE_MODE);

	/* Init endpoints structures */
	for (i = 0U; i < cfg.dev_endpoints; i++)
	{
	  /* Init ep structure */
	  IN_ep[i].is_in = 1U;	//transmit
	  OUT_ep[i].is_in = 0U;	//receive
	  IN_ep[i].num = i;
	  OUT_ep[i].num = i;
	  IN_ep[i].tx_fifo_num = i;

	  /* Control until ep is activated */
	  IN_ep[i].type = EP_TYPE_CTRL;
	  OUT_ep[i].type = EP_TYPE_CTRL;
	  IN_ep[i].maxpacket = 0x40;
	  OUT_ep[i].maxpacket = 0x40;
	  IN_ep[i].xfer_buff = 0U;
	  OUT_ep[i].xfer_buff = 0U;
	  IN_ep[i].xfer_len = 0U;
	}

	USB_DevInit(USBx, cfg);
	//USB_DevDisconnect(USBx);
	//end: USBD_Init

	//TODO setup()

	return status;
}

//static THD_FUNCTION(bridge_thread, arg)
//{
	/*
	t_hydra_console *con;
	con = arg;
	chRegSetThreadName("USB reader");
	chThdSleepMilliseconds(10);
	uint8_t rx_data[UART_BRIDGE_BUFF_SIZE];
	uint8_t bytes_read;
	mode_config_proto_t* proto = &con->mode->proto;

	while (!hydrabus_ubtn()) {
		if(bsp_uart_rxne(proto->dev_num)) {
			bytes_read = bsp_uart_read_u8_timeout(proto->dev_num,
							      rx_data,
							      UART_BRIDGE_BUFF_SIZE,
							      TIME_US2I(100));
			if(bytes_read > 0) {
				cprint(con, (char *)rx_data, bytes_read);
			}
		} else {
			chThdYield();
		}
	}
	*/
//}

//static void bridge(t_hydra_console *con)
//{
	/*
	uint8_t tx_data[USB_BRIDGE_BUFF_SIZE];
	uint8_t bytes_read;

	mode_config_proto_t* proto = &con->mode->proto;

	cprintf(con, "Interrupt by pressing user button.\r\n");
	cprint(con, "\r\n", 2);

	thread_t *bthread = chThdCreateFromHeap(NULL, CONSOLE_WA_SIZE, "bridge_thread",
						LOWPRIO, bridge_thread, con);
	while(!hydrabus_ubtn()) {
		bytes_read = chnReadTimeout(con->, tx_data,
					    UART_BRIDGE_BUFF_SIZE, TIME_US2I(100));
		if(bytes_read > 0) {
			bsp_uart_write_u8(proto->dev_num, tx_data, bytes_read);
		}
	}
	chThdTerminate(bthread);
	chThdWait(bthread);
	*/
//}



bsp_status_t bsp_usb_read(t_hydra_console *con, bsp_dev_usb_t dev_num, uint8_t* dst, uint8_t nb_data)
{
	USB_DevConnect(USBx);
    bsp_status_t status;

	while (1){
		if ((USBx->GINTSTS & USB_OTG_GINTSTS_RXFLVL) != 0) {
			cprintf(con, "\r\n\r\n\r\n***************INCOMING PACKET***************\r\n");
			volatile unsigned int rxst = USBx->GRXSTSP;
			//volatile unsigned int len = (rxst & USB_OTG_GRXSTSP_BCNT) >> 4;

			cprintf(con, "STATUS:\t%x\r\n",(rxst & USB_OTG_GRXSTSP_PKTSTS) >> 17);
			//cprintf(con, "LEN:\t%x\r\n", len);

			if (((rxst & USB_OTG_GRXSTSP_PKTSTS) >> 17) == STS_DATA_UPDT) {
				cprintf(con, "DATA\r\n");
				//(void)USB_ReadPacket(USBx, &setup,len);
			}
			else if (((rxst & USB_OTG_GRXSTSP_PKTSTS) >> 17) == STS_SETUP_UPDT)
			{
				cprintf(con, "SETUP\r\n");
				(void)USB_ReadPacket(USBx, &setup, 8U);

				//if something is received
				if ((USBx_OUTEP(0)->DOEPINT & USB_OTG_DOEPINT_STUP) != 0)
				{
					/*
					//DEBUG
					cprintf(con, "bmRequestType\t%02x\r\n", setup.b.bmRequestType);
					cprintf(con, "bRequest\t%02x\r\n", setup.b.bRequest);
					cprintf(con, "Descriptor Index\t%02x\r\n", setup.b.wValue.bw.msb);
					cprintf(con, "bDescriptorType\t%02x\r\n", setup.b.wValue.bw.lsb);
					cprintf(con, "wValue\t%04x\r\n", setup.b.wValue.w);
					cprintf(con, "wIndex\t%04x\r\n", setup.b.wIndex.w);
					cprintf(con, "wLength\t%04x\r\n", setup.b.wLength.w);
					 */
					switch(setup.b.bRequest)
					{	//bRequest
						case USB_REQ_SET_ADDRESS:	//0x05
							cprintf(con, "*** SET ADDRESS %x\r\n", setup.b.wValue.w);
							//set device address
							USBx_DEVICE->DCFG |= ((setup.b.wValue.w & 0x7f) << 4);
							// IN-NAK
							USBx_INEP(0)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
							break;
						case USB_REQ_GET_DESCRIPTOR: //0x06
							switch (setup.b.wValue.bw.lsb)
							{ //bmRequestType
								case USB_DESC_TYPE_DEVICE: //0x01
									cprintf(con, "\r\n Writing Packet DEV DESC #1 %d\r\n", sizeof(device_desc));
									USB_WritePacket_x(device_desc, MIN(sizeof(device_desc), setup.b.wLength.w), 0);
									break;
								case USB_DESC_TYPE_CONFIGURATION: //0x02
									cprintf(con, "\r\n Writing Packet DEV DESC #2\r\n");
									USB_WritePacket_x(configuration_desc, MIN(sizeof(configuration_desc),setup.b.wLength.w), 0);
									break;
								case USB_DESC_TYPE_STRING:
									cprintf(con, "\r\n Writing Packet DEV DESC #3\r\n");
									switch (setup.b.wValue.bw.msb)
									{
										case STRING_OFFSET_LANGID:
											cprintf(con, "\r\n Writing Packet STR LANGID\r\n");
											USB_WritePacket_x(string_language_desc, MIN(sizeof(string_language_desc), setup.b.wLength.w),0);
											break;
										case STRING_OFFSET_IMANUFACTURER:
											cprintf(con, "\r\n Writing Packet STR MANU\r\n");
											USB_WritePacket_x(string_manufacturer_desc, MIN(sizeof(string_manufacturer_desc), setup.b.wLength.w), 0);
											break;
										case STRING_OFFSET_IPRODUCT:
											cprintf(con, "\r\n Writing Packet STR PROD\r\n");
											USB_WritePacket_x(string_product_desc, MIN(sizeof(string_product_desc), setup.b.wLength.w), 0);
											break;
										case STRING_OFFSET_ISERIAL:
											cprintf(con, "\r\n Writing Packet STR SERIAL\r\n");
											USB_WritePacket_x(string_serial_desc, MIN(sizeof(string_serial_desc), setup.b.wLength.w), 0);
											break;
										case STRING_OFFSET_ICONFIGURATION:
											cprintf(con, "\r\n Writing Packet STR offset...\r\n");
											USB_WritePacket_x(string_configuration_desc, MIN(sizeof(string_configuration_desc), setup.b.wLength.w), 0);
											break;
										default:
											cprintf(con, "\r\n default shit 2\r\n");
											USB_WritePacket_x(0, 0, 0, 0);
											USBx_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK;
									}
									break;
								default:
									cprintf(con, "\r\n default shit 1\r\n");
									USB_WritePacket_x( 0, 0, 0, 0);
									USBx_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK;
							}
							break;
						default:
							cprintf(con, "\r\n default shit 0\r\n");
							USB_WritePacket_x( 0, 0, 0, 0);
							USBx_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK;
					}
				}
			}
		}
	}
	return status;
}


bsp_status_t bsp_usb_write(bsp_dev_usb_t dev_num, uint8_t* src, uint8_t ch_ep_num, uint16_t len)
{
	uint32_t status;

	return status;
}

/**
  * @brief  De-initialize the USB communication bus
  * @param  dev_num: USB dev num.
  * @retval status: status of the deinit.
  */
bsp_status_t bsp_usb_deinit(bsp_dev_usb_t dev_num)
{
	bsp_status_t status;
    USB_DevDisconnect(USBx);
    USB_StopDevice(USBx);

    __HAL_RCC_USB_OTG_HS_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_14|GPIO_PIN_15);

    return status;
}

void USB_WritePacket_x(const void *src, uint16_t len, uint32_t ep) {
	uint8_t numpacket = (len + (0x40 - 1U)) / 0x40;

	uint32_t count32b = 0;
	count32b = (len + 3U) / 4U;

	USBx_INEP(0)->DIEPTSIZ = ((numpacket << 19) & USB_OTG_DIEPTSIZ_PKTCNT) |
							  (len               & USB_OTG_DIEPTSIZ_XFRSIZ);
	USBx_INEP(0)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);


	const uint32_t *src_copy = (const uint32_t *)src;

	uint32_t i = 0;

	for (i = 0; i < count32b; i++) {
	  USBx_DFIFO(0) = *src_copy;
	  src_copy++;
	}

	USBx_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK;

}
