/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <soc/qcom/lge/lge_uart_control.h>

static unsigned int uart_console_mode;

unsigned int lge_get_uart_mode(void)
{
	return uart_console_mode;
}
EXPORT_SYMBOL(lge_get_uart_mode);

void lge_set_uart_mode(unsigned int um)
{
	uart_console_mode = um;
	pr_info("%s: mode : 0x%x\n", __func__, um);
}

static int __init lge_uart_mode(char *uart_mode)
{
	if (!strncmp("enable", uart_mode, 6)) {
		pr_info("UART CONSOLE : enabled\n");
		lge_set_uart_mode((UART_MODE_ALWAYS_ON_BMSK | UART_MODE_EN_BMSK)
						  & ~UART_MODE_ALWAYS_OFF_BMSK);
	} else if (!strncmp("detective", uart_mode, 9)) {
		pr_info("UART CONSOLE : detective\n");
		lge_set_uart_mode(UART_MODE_EN_BMSK & ~UART_MODE_ALWAYS_OFF_BMSK);
	} else {
		pr_info("UART CONSOLE : disabled\n");
		lge_set_uart_mode(UART_MODE_ALWAYS_OFF_BMSK);
	}

	return 1;
}
__setup("lge.uart=", lge_uart_mode);
