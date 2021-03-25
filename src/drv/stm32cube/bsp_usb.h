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
#ifndef _BSP_USB_H_
#define _BSP_USB_H_

#include "bsp.h"
#include "mode_config.h"
#include "hydrabus_mode_usb.h"


typedef enum {
	BSP_DEV_USB1 = 1,
	BSP_DEV_USB2 = 0,
	BSP_DEV_USB_END = 1
} bsp_dev_usb_t;


bsp_status_t bsp_usb_init(t_hydra_console *con, bsp_dev_usb_t dev_num, mode_config_proto_t* mode_conf, uint8_t interface, uint8_t mode);
bsp_status_t bsp_usb_deinit(t_hydra_console *con, bsp_dev_usb_t dev_num);
bsp_status_t bsp_usb_read(t_hydra_console *con, bsp_dev_usb_t dev_num, uint8_t* dst, uint8_t nb_data);
bsp_status_t bsp_usb_write(bsp_dev_usb_t dev_num);
bsp_status_t bsp_usb_setup(t_hydra_console *con);
bsp_status_t bsp_usb_bridge(t_hydra_console *con);

#endif /* _BSP_USB_H_ */
