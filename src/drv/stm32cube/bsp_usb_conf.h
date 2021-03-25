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

#ifndef _BSP_USB_CONF_H_
#define _BSP_USB_CONF_H_

/* USB1 */
#define BSP_USB_OTG_FS        BSP_DEV_USB1
#define BSP_USB_OTG_FS_SPEED  GPIO_SPEED_HIGH
#define BSP_USB_OTG_FS_AF     GPIO_AF10_OTG_HS
#define BSP_USB_OTG_FS_PORT   GPIOA
#define BSP_USB_OTG_FS_DP     GPIO_PIN_12   /* PA.12 */
#define BSP_USB_OTG_FS_DM     GPIO_PIN_11	/* PA.11 */
#define BSP_USB_OTG_FS_ID     GPIO_PIN_10   /* PA.10 */
#define BSP_USB_OTG_FS_VBUS   GPIO_PIN_9    /* PA.09 */
#define BSP_USB_OTG_FS_ITFACE PCD_PHY_EMBEDDED

/* USB2 */
#define BSP_USB_OTG_HS        BSP_DEV_USB2
#define BSP_USB_OTG_HS_SPEED  GPIO_SPEED_HIGH
#define BSP_USB_OTG_HS_AF     GPIO_AF12_OTG_HS_FS
#define BSP_USB_OTG_HS_PORT   GPIOB
#define BSP_USB_OTG_HS_DP     GPIO_PIN_15   /* PB.15 */
#define BSP_USB_OTG_HS_DM     GPIO_PIN_14	/* PB.14 */
#define BSP_USB_OTG_HS_VBUS   GPIO_PIN_13   /* PB.13 */
#define BSP_USB_OTG_HS_ID     GPIO_PIN_12   /* PB.12 */
#define BSP_USB_OTG_HS_ITFACE USB_OTG_EMBEDDED_PHY

#endif /* _BSP_USB_CONF_H_ */
