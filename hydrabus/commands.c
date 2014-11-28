/*
 * HydraBus/HydraNFC
 *
 * Copyright (C) 2014 Bert Vermeulen <bert@biot.com>
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

#include "tokenline.h"
#include "commands.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

t_token_dict tl_dict[] = {
	{ /* Dummy entry */ },
	{ T_HELP, "help" },
	{ T_HISTORY, "history" },
	{ T_EXIT, "exit" },
	{ T_CLEAR, "clear" },
	{ T_DEBUG, "debug" },
	{ T_SHOW, "show" },
	{ T_SYSTEM, "system" },
	{ T_MEMORY, "memory" },
	{ T_THREADS, "threads" },
	{ T_SD, "sd" },
	{ T_MOUNT, "mount" },
	{ T_UMOUNT, "umount" },
	{ T_CD, "cd" },
	{ T_PWD, "pwd" },
	{ T_LS, "ls" },
	{ T_CAT, "cat" },
	{ T_HD, "hd" },
	{ T_ERASE, "erase" },
	{ T_REALLY, "really" },
	{ T_TESTPERF, "test-perf" },
	{ T_MODE, "mode" },
	{ T_SPI, "spi" },
	{ T_I2C, "i2c" },
	{ T_DEVICE, "device" },
	{ T_MASTER, "master" },
	{ T_SLAVE, "slave" },
	{ T_FREQUENCY, "frequency" },
	{ T_POLARITY, "polarity" },
	{ T_PHASE, "phase" },
	{ T_MSB_FIRST, "msb-first" },
	{ T_LSB_FIRST, "lsb-first" },
	{ T_GPIO_RESISTOR, "gpio-resistor" },
	{ T_PULL_UP, "pull-up" },
	{ T_PULL_DOWN, "pull-down" },
	{ T_FLOATING, "floating" },
	{ T_ON, "on" },
	{ T_OFF, "off" },
	{ T_CHIP_SELECT, "chip-select" },
	{ T_CS, "cs" },
	{ T_PINS, "pins" },
	{ T_READ, "read" },
	{ T_WRITE, "write" },
	{ T_LEFT_SQ, "[" },
	{ T_RIGHT_SQ, "]" },
	{ T_LEFT_CURLY, "{" },
	{ T_RIGHT_CURLY, "}" },
	{ T_SLASH, "/" },
	{ T_BACKSLASH, "\\" },
	{ T_MINUS, "-" },
	{ T_UNDERSCORE, "_" },
	{ T_EXCLAMATION, "!" },
	{ T_CARET, "^" },
	{ T_PERIOD, "." },
	{ T_AMPERSAND, "&" },
	{ T_PERCENT, "%" },
	{ }
};

t_token tokens_on_off[] = {
	{ T_ON },
	{ T_OFF },
	{ }
};

t_token tokens_master_slave[] = {
	{ T_MASTER },
	{ T_SLAVE },
	{ }
};

t_token tokens_gpio_resistance[] = {
	{ T_PULL_UP },
	{ T_PULL_DOWN },
	{ T_FLOATING },
	{ }
};

#define SPI_PARAMETERS \
	{ T_DEVICE, \
		.arg_type = T_ARG_INT, \
		.help = "SPI device (1 or 2)" }, \
	{ T_GPIO_RESISTOR, \
		.arg_type = T_ARG_TOKEN, \
		.subtokens = tokens_gpio_resistance, \
		.help = "GPIO resistance" }, \
	{ T_MODE, \
		.arg_type = T_ARG_TOKEN, \
		.subtokens = tokens_master_slave, \
			.help = "Master or slave mode" }, \
	{ T_FREQUENCY, \
		.arg_type = T_ARG_FREQ, \
		.help = "Read/write frequency" }, \
	{ T_POLARITY, \
		.arg_type = T_ARG_INT, \
		.help = "Clock polarity (0 or 1)" }, \
	{ T_PHASE, \
		.arg_type = T_ARG_INT, \
		.help = "Clock phase (0 or 1)" }, \
	{ T_MSB_FIRST, \
		.help = "Send/receive MSB first" }, \
	{ T_LSB_FIRST, \
		.help = "Send/receive LSB first" },

t_token tokens_mode_show[] = {
	{ T_PINS,
		.help = "Show SPI pins" },
	{ }
};

t_token tokens_mode_spi[] = {
	{ T_SHOW,
		.subtokens = tokens_mode_show,
		.help = "Show SPI parameters" },
	SPI_PARAMETERS
	/* SPI-specific commands */
	{ T_CHIP_SELECT,
		.arg_type = T_ARG_TOKEN,
		.subtokens = tokens_on_off,
		.help = "Chip select (CS)" },
	{ T_CS,
		.arg_type = T_ARG_TOKEN,
		.subtokens = tokens_on_off,
		.help = "Alias for \"chip-select\"" },
	{ T_READ,
		.flags = T_FLAG_SUFFIX_TOKEN_DELIM_INT,
		.help = "Read byte (repeat with :<num>)" },
	{ T_WRITE,
		.arg_type = T_ARG_INT,
		.flags = T_FLAG_SUFFIX_TOKEN_DELIM_INT,
		.help = "Write byte (repeat with :<num>)" },
	{ T_ARG_INT,
		.help = "Write byte (repeat with :<num>)" },
	/* BP commands */
	{ T_LEFT_SQ,
		.help = "Alias for \"chip-select on\"" },
	{ T_RIGHT_SQ,
		.help = "Alias for \"chip-select off\"" },
	{ T_AMPERSAND,
		.flags = T_FLAG_SUFFIX_TOKEN_DELIM_INT,
		.help = "Delay 1 usec (repeat with :<num>)" },
	{ T_PERCENT,
		.flags = T_FLAG_SUFFIX_TOKEN_DELIM_INT,
		.help = "Delay 1 msec (repeat with :<num>)" },
	{ T_EXIT,
		.help = "Exit SPI mode" },
	{ }
};

t_token tokens_spi[] = {
	SPI_PARAMETERS
	{ }
};

t_token tokens_modes[] = {
	{ T_SPI,
		.subtokens = tokens_spi,
		.help = "SPI mode" },
	{ T_I2C,
		.subtokens = tokens_spi,
		.help = "I2C mode" },
	{ }
};

t_token tokens_really[] = {
	{ T_REALLY },
	{ }
};

t_token tokens_sd[] = {
	{ T_MOUNT,
		.help = "Mount SD card" },
	{ T_UMOUNT,
		.help = "Unmount SD card" },
	{ T_ERASE,
		.help = "Erase and reformat SD card" },
	{ T_CD,
		.arg_type = T_ARG_STRING,
		.help = "Change SD card directory" },
	{ T_PWD,
		.help = "Show current SD card directory" },
	{ T_LS,
		.help = "List files on SD card" },
	{ T_TESTPERF,
		.help = "Test SD card performance" },
	{ T_CAT,
		.arg_type = T_ARG_STRING,
		.help = "Display (ASCII) file on SD card" },
	{ T_HD,
		.arg_type = T_ARG_STRING,
		.help = "Hexdump file on SD card" },
	{ }
};

t_token tokens_show[] = {
	{ T_SYSTEM },
	{ T_MEMORY },
	{ T_THREADS },
	{ }
};

t_token tl_tokens[] = {
	{ T_HELP,
			.arg_type = T_ARG_HELP,
		.help = "Available commands" },
	{ T_HISTORY,
		.help = "Command history" },
	{ T_CLEAR,
		.help = "Clear screen" },
	{ T_SHOW,
		.subtokens = tokens_show,
		.help = "Show information" },
	{ T_SD,
		.subtokens = tokens_sd,
		.help = "SD card management" },
	{ T_MODE,
		.subtokens = tokens_modes,
		.help = "Switch to protocol mode" },
	{ T_DEBUG,
		.help = "Debug mode" },
	{ }
};

#pragma GCC diagnostic pop
