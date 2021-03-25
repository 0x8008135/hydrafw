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

#include "bsp_usb.h"
#include "bsp_usb_conf.h"
#include "stm32f405xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_pcd.h"
#include "stm32f4xx_hal_pcd_ex.h"
#include "hydrabus_mode_usb.h"

#define USB_DESC_TYPE_DEVICE                           0x01
#define USB_DESC_TYPE_CONFIGURATION                    0x02
#define USB_DESC_TYPE_STRING                           0x03
#define USB_DESC_TYPE_INTERFACE                        0x04
#define USB_DESC_TYPE_ENDPOINT                         0x05
#define USB_DESC_TYPE_DEVICE_QUALIFIER                 0x06
#define USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION        0x07
#define USB_DESC_TYPE_BINARY_OBJECT_STORE              0x0f

// offsets for configuration strings
#define STRING_OFFSET_LANGID                           0x00
#define STRING_OFFSET_IMANUFACTURER                    0x01
#define STRING_OFFSET_IPRODUCT                         0x02
#define STRING_OFFSET_ISERIAL                          0x03
#define STRING_OFFSET_ICONFIGURATION                   0x04
#define STRING_OFFSET_IINTERFACE                       0x05
#define UART_BRIDGE_BUFF_SIZE                          0x40

#define STRING_DESCRIPTOR_HEADER(size)(((((size) * 2) + 2) & 0xFF) | 0x0300)

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
    0x0a, 0x01, //Length, Type
    0x10, 0x02, // bcdUSB max version of USB supported (2.1)
    0xFF, 0xFF, //
    0xFF, 0x40, // bDeviceClass, bDeviceSubClass, bDeviceProtocol, bMaxPacketSize0
    0x01, 0x00 	// bNumConfigurations, bReserved
};

uint16_t string_language_desc[] = {
    STRING_DESCRIPTOR_HEADER(1),
	0x0409
};

uint16_t string_manufacturer_desc[] = {
    STRING_DESCRIPTOR_HEADER(14),
    'O', 'p', 'e', 'n', 'm', 'o', 'k', 'o', ',', ' ', 'I', 'n', 'c', '.'
};

uint16_t string_product_desc[] = {
    STRING_DESCRIPTOR_HEADER(24),
    'H', 'y', 'd', 'r', 'a', 'B', 'u', 's', ' ', '1', '.', '0', ' ', 'C', 'O', 'M', ' ', 'P',  'o', 'r', 't', '2'
};

uint16_t string_serial_desc[] = {
    STRING_DESCRIPTOR_HEADER(26),
    '0', '0', '3', '7', '0', '0', '1', 'E', '3', '4', '3', '6', '5', '1', '1', '1', '3', '9', '3', '3', '3', '4', '3', '4'
};

uint16_t string_configuration_desc[] = {
    STRING_DESCRIPTOR_HEADER(2),
    '0', '1'
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
} USB_Setup_TypeDef;

USB_Setup_TypeDef setup;

PCD_HandleTypeDef hpcd_fs;
PCD_HandleTypeDef hpcd;

PCD_EPTypeDef in_ep;
PCD_EPTypeDef out_ep;

#define NB_USB (BSP_DEV_USB_END)

//static USBD_HandleTypeDef usbd_handle[NB_USB];
//static mode_config_proto_t* usbd_mode_conf[NB_USB];



/**
  * @brief  Init low level hardware: GPIO, CLOCK, NVIC...
  * @param  dev_num: USB dev num
  * @retval None
  */
static void usb_gpio_hw_init(bsp_dev_usb_t dev_num)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	if(dev_num == BSP_DEV_USB1) {
        /* USB1 => USB_OTG_FS */
        /* Enable the USB_OTG_FS peripheral */
        __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

        /* Enable GPIOA Clock */
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* Configure VBUS Pin */
        GPIO_InitStructure.Pin = BSP_USB_OTG_FS_VBUS;
        GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
        GPIO_InitStructure.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(BSP_USB_OTG_FS_PORT, &GPIO_InitStructure);
        
        /* Configure ID pin */
        GPIO_InitStructure.Pin = BSP_USB_OTG_FS_ID;
        GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStructure.Pull = GPIO_PULLUP;
        GPIO_InitStructure.Alternate = BSP_USB_OTG_FS_AF;
        HAL_GPIO_Init(BSP_USB_OTG_FS_PORT, &GPIO_InitStructure);

        /* Configure FS_DM / FS_DP */
        GPIO_InitStructure.Pin = BSP_USB_OTG_FS_DM|BSP_USB_OTG_FS_DP;
        GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStructure.Pull = GPIO_NOPULL;
        GPIO_InitStructure.Speed = BSP_USB_OTG_FS_SPEED;
        GPIO_InitStructure.Alternate = BSP_USB_OTG_FS_AF;
        HAL_GPIO_Init(BSP_USB_OTG_FS_PORT, &GPIO_InitStructure);

	} else {
        /* USB2 => USB_OTG_HS */
        /* Enable the USB_OTG_HS peripheral */
        __HAL_RCC_USB_OTG_HS_CLK_ENABLE();

        /* Enable GPIOB Clock */
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* Configure VBUS Pin */
        GPIO_InitStructure.Pin = BSP_USB_OTG_HS_VBUS;
        GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
        GPIO_InitStructure.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(BSP_USB_OTG_HS_PORT, &GPIO_InitStructure);

        /* Configure ID pin */
        GPIO_InitStructure.Pin = BSP_USB_OTG_HS_ID;
        GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStructure.Pull = GPIO_PULLUP;
        GPIO_InitStructure.Alternate = BSP_USB_OTG_HS_AF;
        HAL_GPIO_Init(BSP_USB_OTG_HS_PORT, &GPIO_InitStructure);

        /* Configure HS_DM / HS_DP */
        GPIO_InitStructure.Pin = BSP_USB_OTG_HS_DM|BSP_USB_OTG_HS_DP;
        GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStructure.Pull = GPIO_NOPULL;
        GPIO_InitStructure.Speed = BSP_USB_OTG_HS_SPEED;
        GPIO_InitStructure.Alternate = BSP_USB_OTG_HS_AF;
        HAL_GPIO_Init(BSP_USB_OTG_HS_PORT, &GPIO_InitStructure);

	}

}

/**
  * @brief  DeInit low level hardware: GPIO, CLOCK, NVIC...
  * @param  dev_num: USB dev num
  * @retval None
  */
static void usb_gpio_hw_deinit(bsp_dev_usb_t dev_num)
{
	if(dev_num == BSP_DEV_USB1) {
        /* USB1 => USB_OTG_FS */
		/* Reset peripherals */
        __HAL_RCC_USB_OTG_FS_FORCE_RESET();
        __HAL_RCC_USB_OTG_FS_RELEASE_RESET();
        
		/* Disable peripherals GPIO */
        HAL_GPIO_DeInit(BSP_USB_OTG_FS_PORT, BSP_USB_OTG_FS_VBUS);
        HAL_GPIO_DeInit(BSP_USB_OTG_FS_PORT, BSP_USB_OTG_FS_ID);
        HAL_GPIO_DeInit(BSP_USB_OTG_FS_PORT, BSP_USB_OTG_FS_DM|BSP_USB_OTG_FS_DP);
        
        __HAL_RCC_GPIOA_CLK_DISABLE();
        
	} else {
        /* USB2 => USB_OTG_HS */
		/* Reset peripherals */
        __HAL_RCC_USB_OTG_HS_FORCE_RESET();
        __HAL_RCC_USB_OTG_HS_RELEASE_RESET();

		/* Disable peripherals GPIO */
        HAL_GPIO_DeInit(BSP_USB_OTG_HS_PORT, BSP_USB_OTG_HS_VBUS);
        HAL_GPIO_DeInit(BSP_USB_OTG_HS_PORT, BSP_USB_OTG_HS_ID);
        HAL_GPIO_DeInit(BSP_USB_OTG_HS_PORT, BSP_USB_OTG_HS_DM|BSP_USB_OTG_HS_DP);

        __HAL_RCC_GPIOB_CLK_DISABLE();
	}
}

/**
 * @brief  Init USB device.
 * @param  dev_num: USB dev num.
 * @param  mode_conf: Mode config proto.
 * @retval status: status of the init.
 */

bsp_status_t bsp_usb_init(t_hydra_console *con, bsp_dev_usb_t dev_num, mode_config_proto_t* mode_conf, uint8_t interface, uint8_t mode)
{
    bsp_status_t status;
    uint8_t i;

    if (interface == 0) {
        hpcd.Instance = USB_OTG_HS;
    } else {
        hpcd.Instance = USB_OTG_FS;
    }
    /* Init the low level hardware : GPIO, CLOCK, NVIC... */
    usb_gpio_hw_init(interface);
    //host == 1 / device == 0
    if (mode == 1U) {
        status = 1;
        cprintf(con, "\r\nHost mode not implemented...yet\r\n");
        return BSP_ERROR;
    } else {
        cprintf(con, "\r\nDevice mode...\r\n");
        /* USB Address unconfigured */
        hpcd.USB_Address = 0U;
        /* Device Configuration */
        hpcd.Init.dev_endpoints = 4;
        hpcd.Init.dma_enable = DISABLE;
        hpcd.Init.ep0_mps = 0x40;
        hpcd.Init.low_power_enable = DISABLE;
        hpcd.Init.lpm_enable = DISABLE;
        hpcd.Init.phy_itface = BSP_USB_OTG_HS_ITFACE;
        hpcd.Init.Sof_enable = DISABLE;
        hpcd.Init.speed = PCD_SPEED_FULL;
        hpcd.Init.use_dedicated_ep1 = DISABLE;
        hpcd.Init.use_external_vbus = DISABLE;
        hpcd.Init.vbus_sensing_enable = DISABLE; // Cannot use vbus_sensing as FS & HS are always on
        hpcd.Init.battery_charging_enable = DISABLE;
        /*Init the Core (common init.) */
        if (USB_CoreInit(hpcd.Instance, hpcd.Init) != HAL_OK) {
            return BSP_ERROR;
        }
        
        /* Force Device Mode*/
        if (USB_SetCurrentMode(hpcd.Instance, USB_DEVICE_MODE)!= HAL_OK) {
            return BSP_ERROR;
        }
        /* Init endpoints structures */
        for (i = 0U; i < hpcd.Init.dev_endpoints; i++) {
            /* Init ep structure */
            hpcd.IN_ep[i].is_in = 1U;
            hpcd.IN_ep[i].num = i;
            hpcd.IN_ep[i].tx_fifo_num = i;
            /* Control until ep is activated */
            hpcd.IN_ep[i].type = EP_TYPE_CTRL;
            hpcd.IN_ep[i].maxpacket = 0U;
            hpcd.IN_ep[i].xfer_buff = 0U;
            hpcd.IN_ep[i].xfer_len = 0U;
        }
        
        for (i = 0U; i < hpcd.Init.dev_endpoints; i++) {
            hpcd.OUT_ep[i].is_in = 0U;
            hpcd.OUT_ep[i].num = i;
            /* Control until ep is activated */
            hpcd.OUT_ep[i].type = EP_TYPE_CTRL;
            hpcd.OUT_ep[i].maxpacket = 0U;
            hpcd.OUT_ep[i].xfer_buff = 0U;
            hpcd.OUT_ep[i].xfer_len = 0U;
        }
        
        /* Init Device */
        if (USB_DevInit(hpcd.Instance, hpcd.Init) != HAL_OK) {
            return BSP_ERROR;
        }
        HAL_PCDEx_SetRxFiFo(&hpcd, 0x40);
        HAL_PCDEx_SetTxFiFo(&hpcd, 0, 0x40); // EP0 IN
        if (USB_DevDisconnect(hpcd.Instance)!= HAL_OK) {
            return BSP_ERROR;
        }
        bsp_usb_setup(con);
    }

    return status;
}

bsp_status_t bsp_usb_setup(t_hydra_console *con)
{
    //static void usb_setup(t_hydra_console *con) {
    bsp_status_t status;
    status = BSP_OK;
    
    USB_OTG_GlobalTypeDef *USBx = hpcd.Instance;
    USB_OTG_EPTypeDef *ep_out = &hpcd.OUT_ep[0];
    USB_OTG_EPTypeDef *ep_in = &hpcd.IN_ep[0];
    
    //TODO
    //FIX maxpacket size ?
    ep_in->maxpacket = 0x40;
    ep_out->maxpacket = 0x40;

    (void)USB_DevConnect(USBx);

    while (1) {
		/* Exit if User Button is pressed */
		if (hydrabus_ubtn()) {
			break;
		}
        if ((USBx->GINTSTS & USB_OTG_GINTSTS_RXFLVL) != 0) {
            if (((USBx->GRXSTSP & USB_OTG_GRXSTSP_PKTSTS) >> 17) ==  STS_SETUP_UPDT)
            {//IF SETUP
                (void)USB_ReadPacket(USBx, &setup, 8U);
                //SWITCH bRequest
                switch(setup.b.bRequest) {
                    case USB_REQ_SET_ADDRESS:	//0x05
                        cprintf(con, "*** SET ADDRESS %d\r\n", setup.b.wValue.w);
                        hpcd.USB_Address = setup.b.wValue.w;
                        ep_in->xfer_buff = 0;
                        ep_in->xfer_len = 0;
                        USB_SetDevAddress(USBx, setup.b.wValue.w);
                        USB_EPStartXfer(USBx, ep_in, 0);
                        break;
                    case USB_REQ_GET_DESCRIPTOR: //0x06
                        //SWITCH value lsb
                        switch (setup.b.wValue.bw.lsb) {
                            //bmRequestType
                            case USB_DESC_TYPE_DEVICE: //0x01
                                cprintf(con, "\r\n[USB] Writing Packet DEV DESC #1\r\n");
                                ep_in->xfer_count = 0;
                                ep_in->xfer_buff = &device_desc;
                                ep_in->xfer_len  = MIN(sizeof(device_desc),setup.b.wLength.w);        
                                USB_EPStartXfer(USBx, ep_in, 0);            
                                USB_WritePacket(USBx, ep_in->xfer_buff, 0, ep_in->xfer_len, 0);
                                USB_EPStartXfer(USBx, ep_out, 0);
                                break;
                            case USB_DESC_TYPE_CONFIGURATION: //0x02
                                cprintf(con, "\r\n[USB] Writing Packet DEV DESC #2\r\n");
                                ep_in->xfer_count = 0;
                                ep_in->xfer_buff = &configuration_desc;
                                ep_in->xfer_len  = MIN(sizeof(configuration_desc),setup.b.wLength.w);        
                                USB_EPStartXfer(USBx, ep_in, 0);
                                USB_WritePacket(USBx, ep_in->xfer_buff, 0, ep_in->xfer_len, 0);
                                USB_EPStartXfer(USBx, ep_out, 0);
                                break;
                            case USB_DESC_TYPE_STRING:
                                cprintf(con, "\r\n[USB] Writing Packet DEV DESC #3\r\n");
                                //SWITCH value msb
                                switch (setup.b.wValue.bw.msb) {
                                    case STRING_OFFSET_LANGID:
                                        cprintf(con, "\r\n Writing Packet STR LANGID\r\n");
                                        ep_in->xfer_count = 0;
                                        ep_in->xfer_buff = &string_language_desc;
                                        ep_in->xfer_len  = MIN(sizeof(string_language_desc), setup.b.wLength.w);        
                                        USB_EPStartXfer(USBx, ep_in, 0);
                                        USB_WritePacket(USBx, ep_in->xfer_buff, 0, ep_in->xfer_len, 0);
                                        USB_EPStartXfer(USBx, ep_out, 0);
                                        break;
                                    case STRING_OFFSET_IMANUFACTURER:
                                        cprintf(con, "\r\n Writing Packet STR MANU\r\n");
                                        ep_in->xfer_count = 0;
                                        ep_in->xfer_buff = &string_manufacturer_desc;
                                        ep_in->xfer_len  = MIN(sizeof(string_manufacturer_desc), setup.b.wLength.w);        
                                        USB_EPStartXfer(USBx, ep_in, 0);
                                        USB_WritePacket(USBx, ep_in->xfer_buff, 0,  ep_in->xfer_len, 0);
                                        USB_EPStartXfer(USBx, ep_out, 0);
                                        break;
                                    case STRING_OFFSET_IPRODUCT:
                                        cprintf(con, "\r\n Writing Packet STR PROD\r\n");
                                        ep_in->xfer_count = 0;
                                        ep_in->xfer_buff = &string_product_desc;
                                        ep_in->xfer_len  = MIN(sizeof(string_product_desc), setup.b.wLength.w);        
                                        USB_EPStartXfer(USBx, ep_in, 0);
                                        USB_WritePacket(USBx, ep_in->xfer_buff, 0, ep_in->xfer_len, 0);
                                        USB_EPStartXfer(USBx, ep_out, 0);
                                        break;
                                    case STRING_OFFSET_ISERIAL:
                                        cprintf(con, "\r\n Writing Packet STR SERIAL\r\n");
                                        ep_in->xfer_count = 0;
                                        ep_in->xfer_buff = &string_serial_desc;
                                        ep_in->xfer_len  = MIN(sizeof(string_serial_desc), setup.b.wLength.w);        
                                        USB_EPStartXfer(USBx, ep_in, 0);
                                        USB_WritePacket(USBx, ep_in->xfer_buff, 0, ep_in->xfer_len, 0);
                                        USB_EPStartXfer(USBx, ep_out, 0);
                                        break;
                                    case STRING_OFFSET_ICONFIGURATION:
                                        cprintf(con, "\r\n Writing Packet STR offset...\r\n");
                                        ep_in->xfer_count = 0;
                                        ep_in->xfer_buff = &string_configuration_desc;
                                        ep_in->xfer_len  = MIN(sizeof(string_configuration_desc), setup.b.wLength.w);        
                                        USB_EPStartXfer(USBx, ep_in, 0);
                                        USB_WritePacket(USBx, ep_in->xfer_buff, 0, ep_in->xfer_len, 0);
                                        USB_EPStartXfer(USBx, ep_out, 0);
                                        break;
                                    default:
                                        ep_in->xfer_count = 0;
                                        ep_in->xfer_buff = NULL;
                                        ep_in->xfer_len  = 0;  
                                        USB_EPStartXfer(USBx, ep_in, 0);
                                }//END SWITCH value msb
                                break;
                            default:
                                ep_in->xfer_count = 0;
                                ep_in->xfer_buff = NULL;
                                ep_in->xfer_len  = 0;  
                                USB_EPStartXfer(USBx, ep_in, 0);
                        }//END SWITCH value lsb
                        break;
                    default:
                        ep_in->xfer_count = 0;
                        ep_in->xfer_buff = NULL;
                        ep_in->xfer_len  = 0;  
                        USB_EPStartXfer(USBx, ep_in, 0);
                }//END SWITCH bRequest
            }//END IF 
        }
    }//END WHILE

    return status;
}

bsp_status_t bsp_usb_bridge(t_hydra_console *con)
{
    bsp_status_t status;
    status = BSP_OK;
    
    USB_OTG_GlobalTypeDef *USB1 = USB_OTG_FS;
    USB_OTG_GlobalTypeDef *USB2 = USB_OTG_HS;
    
    //USB_OTG_EPTypeDef *ep_out = &hpcd.OUT_ep[0];
    //USB_OTG_EPTypeDef *ep_in = &hpcd.IN_ep[0];
    USB_OTG_EPTypeDef * ep;
    USB_OTG_EPTypeDef toto;
    ep = &toto;

    int endpoint, len;

    uint8_t buff[255];

    //TODO
    //FIX maxpacket size ?
    ep->maxpacket = 0x40;
    //ep->out.maxpacket = 0x40;
    
    (void)USB_DevDisconnect(USB1);
    (void)USB_DevDisconnect(USB2);
    (void)USB_DevConnect(USB1);
    (void)USB_DevConnect(USB2);
    

    while (1) {
		/* Exit if User Button is pressed */
		if (hydrabus_ubtn()) {
			break;
		}
        //IF packet received on USB2
        if ((USB2->GINTSTS & USB_OTG_GINTSTS_RXFLVL) != 0) {
            //get endpoint
            endpoint = (USB2->GRXSTSP & USB_OTG_GRXSTSP_EPNUM);
            //ep->num = endpoint;

            //get len data
            len = (USB2->GRXSTSP & USB_OTG_GRXSTSP_BCNT) >> 4;
            //ep->xfer_len = len;
            (void)USB_ReadPacket(USB2, &buff, len);
            cprintf(con, "**USB2**");
            cprintf(con, "\r\nEP   : %d\r\n", endpoint);
            cprintf(con, "SIZE : %d\r\n", len);
            cprintf(con, "PCKT : ");
            int i=0;
            for(i = 0; i<len; i++)
                cprintf(con, "%#x ", buff[i]);
            cprintf(con, "\r\n");
            (void)USB_WritePacket(USB1, &buff, endpoint, len, 0);
        }
        //IF packet received on USB1
        if ((USB1->GINTSTS & USB_OTG_GINTSTS_RXFLVL) != 0) {
            //get endpoint
            endpoint = (USB1->GRXSTSP & USB_OTG_GRXSTSP_EPNUM);
            //ep->num = endpoint;

            //get len data
            len = (USB1->GRXSTSP & USB_OTG_GRXSTSP_BCNT) >> 4;
            
            //ep->xfer_len = len;
            (void)USB_ReadPacket(USB1, &buff, len);
            cprintf(con, "**USB1**");
            cprintf(con, "\r\nEP   : %d\r\n", endpoint);
            cprintf(con, "SIZE : %d\r\n", len);
            cprintf(con, "PCKT : ");
            int i=0;
            for(i = 0; i<len; i++)
                cprintf(con, "%#x ", buff[i]);
            cprintf(con, "\r\n");
            (void)USB_WritePacket(USB2, &buff, endpoint, len, 0);
        }
    }
            /*
            if (((USB2->GRXSTSP & USB_OTG_GRXSTSP_PKTSTS) >> 17) ==  STS_SETUP_UPDT)
            {
                (void)USB_ReadPacket(USB2, &setup, len);

                if (setup.b.bRequest == USB_REQ_SET_ADDRESS){
                    cprintf(con, "*** SET ADDRESS %d\r\n", setup.b.wValue.w);
                    USB2.USB_Address = setup.b.wValue.w;
                    ep_in->xfer_buff = 0;
                    ep_in->xfer_len = 0;
                    
                    USB_SetDevAddress(USB2, setup.b.wValue.w);
                    USB_EPStartXfer(USB2, ep_in, 0);
                }
                else
                {
                    (void)USB_ReadPacket(USB2, &buff, len);

                }
            else
            {
                (void)USB_ReadPacket(USB2, &buff, len);
            }
            */
            
            /*cprintf(con, "PACKET : %02x\r\n",buff);

            ep.xfer_count = 0;
            ep.xfer_buff = &buff;
            ep.xfer_len  = len;
            ep->is_in = 1;
            USB_EPStartXfer(USB2, &ep, 0);            
            USB_WritePacket(USB2, &ep.xfer_buff, 0, ep.xfer_len, 0);
            USB_EPStartXfer(USB2, &ep, 0);*/
            //if ((USBy->GINTSTS & USB_OTG_GINTSTS_RXFLVL) != 0) {
    return status;
}

bsp_status_t bsp_usb_read(t_hydra_console *con, bsp_dev_usb_t dev_num, uint8_t* dst, uint8_t nb_data)
{
    bsp_status_t status;
    status = BSP_OK;
    return status;
}

bsp_status_t bsp_usb_write(bsp_dev_usb_t dev_num)
{
    uint32_t status;
    status = BSP_OK;
    return status;
}

/**
 * @brief  De-initialize the USB communication bus
 * @param  dev_num: USB dev num.
 * @retval status: status of the deinit.
 */
bsp_status_t bsp_usb_deinit(t_hydra_console *con, bsp_dev_usb_t dev_num)
{
    bsp_status_t status;

    if (HAL_PCD_Stop(&hpcd)!= HAL_OK)
        cprintf(con, "\r\nHAL PCD STOP KO");
    cprintf(con, "\r\nSTOP OK");

    __HAL_RCC_USB_OTG_HS_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_14|GPIO_PIN_15);

    return status;
}

/**
 * @brief  USBx error treatment function.
 * @param  dev_num: USB dev num
 * @retval None
 */
static void usb_error(bsp_dev_usb_t dev_num)
{
    (void)dev_num;
    //if(bsp_usb_deinit(dev_num) == BSP_OK) {
        /* Re-Initialize the USB communication bus */
        //bsp_usb_init(dev_num, usb_mode_conf[dev_num]);
    //}
}


/*
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
*/