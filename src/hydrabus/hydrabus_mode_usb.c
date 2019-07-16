/*
 * HydraBus/HydraNFC
 *
 * Copyright (C) 2014-2015 Benjamin VERNOUX
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "common.h"
#include "hydrabus_mode_usb.h"
#include "bsp_usb.h"
#include <string.h>

static int exec(t_hydra_console *con, t_tokenline_parsed *p, int token_pos);
static int show(t_hydra_console *con, t_tokenline_parsed *p);

static const char* str_pins_usb[] = {
	"[USB2] ID: PB12\r\nVBUS: PB13\r\nDM: PB14\r\nDM: PB15\r\n"
};
static const char* str_prompt_usb[] = {
	"usb2" PROMPT,
};


//static const char* str_bsp_init_err= { "bsp_usb_init() error %d\r\n" };

static void init_proto_default(t_hydra_console *con)
{
	mode_config_proto_t* proto = &con->mode->proto;

	/* Defaults */
	//FIXME
	(void)proto;
}

static void show_params(t_hydra_console *con)
{
	mode_config_proto_t* proto = &con->mode->proto;
	cprint(con, "\r\n", 2);
	//FIXME
	(void)proto;
}

static int init(t_hydra_console *con, t_tokenline_parsed *p)
{
	mode_config_proto_t* proto = &con->mode->proto;
	int tokens_used;

	/* Defaults */
	init_proto_default(con);

	/* Process cmdline arguments, skipping "usb". */
	tokens_used = 1 + exec(con, p, 1);
    
    bsp_usb_init(proto->dev_num, proto);

	show_params(con);

	return tokens_used;
}

static int exec(t_hydra_console *con, t_tokenline_parsed *p, int token_pos)
{
	mode_config_proto_t* proto = &con->mode->proto;
	//FIXME
	(void)proto;
	int arg_int, t;
	(void)arg_int;
	for (t = token_pos; p->tokens[t]; t++) {
		switch (p->tokens[t]) {
		case T_SHOW:
			t += show(con, p);
			break;
		default:
			return t - token_pos;
		}
	}

	return t - token_pos;
}

static int show(t_hydra_console *con, t_tokenline_parsed *p)
{
	mode_config_proto_t* proto = &con->mode->proto;
	int tokens_used;

	tokens_used = 0;
	if (p->tokens[1] == T_PINS) {
		tokens_used++;
		cprintf(con, "%s", str_pins_usb[proto->dev_num]);
	} else {
		show_params(con);
	}

	return tokens_used;
}

static const char *get_prompt(t_hydra_console *con)
{
	mode_config_proto_t* proto = &con->mode->proto;

	return str_prompt_usb[proto->dev_num];
}

static void cleanup(t_hydra_console *con)
{
     mode_config_proto_t* proto = &con->mode->proto;
 
     bsp_usb_deinit(proto->dev_num);
}

static uint32_t read(t_hydra_console *con, uint8_t *rx_data, uint8_t nb_data)
{
	int i;
	uint32_t status;
	mode_config_proto_t* proto = &con->mode->proto;

	status = bsp_usb_read(con,proto->dev_num, rx_data, nb_data);

	if(status == BSP_OK) {
		if(nb_data == 1) {
			/* Read 1 data */
			cprintf(con, hydrabus_mode_str_read_one_u8, rx_data[0]);
		} else if(nb_data > 1) {
			for(i = 0; i < nb_data; i++) {
				cprintf(con, hydrabus_mode_str_mul_value_u8, rx_data[i]);
			}
			cprintf(con, hydrabus_mode_str_mul_br);
		}
	}

	return status;
}
static uint32_t write(t_hydra_console *con, uint8_t *tx_data, uint8_t nb_data)
{
	int i;
	uint32_t status;
	mode_config_proto_t* proto = &con->mode->proto;

    //FIXME endpoint number
	//status = bsp_usb_write(proto->dev_num, &device_desc , 0, sizeof(device_desc));

	return status;
}
const mode_exec_t mode_usb_exec = {
	.init = &init,
	.exec = &exec,
	.cleanup = &cleanup,
    //.show = &show,
	.read = &read,
	.write = &write,
    .get_prompt = &get_prompt,
};

