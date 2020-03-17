// SPDX-License-Identifier: GPL-2.0-only
/*
 * 8250 serial console setup for the Realtek RTL838X SoC
 * 
 * based on the original BSP by
 * Copyright (C) 2006-2012 Tony Wu (tonywu@realtek.com)
 * 
 * Copyright (C) 2020 B. Koblitz
 * 
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>
#include <linux/serial_reg.h>
#include <linux/tty.h>
#include <linux/clk.h>

#include "mach-rtl838x.h"

extern char arcs_cmdline[];
extern struct clk cpu_clk;

int __init rtl838x_serial_init(void)
{
#ifdef CONFIG_SERIAL_8250
	int ret;
	struct uart_port p;
	u32 cpu_freq = clk_get_rate(&cpu_clk);
	int baud = 0;
	char parity = '\0', bits = '\0', flow = '\0';
	char *s, *n;

	s = strstr(arcs_cmdline, "console=ttyS0,");
	if (s) {
		s += 14;
		baud = simple_strtoul(s, &n, 10);
		s = n;
		if (*s == ',')
			s++;
		if (*s)
			parity = *s++;
		if (*s == ',')
			s++;
		if (*s)
			bits = *s++;
		if (*s == ',')
			s++;
		if (*s == 'h')
			flow = 'r';
	}

	if (baud == 0)
		baud = 38400;
	if (parity != 'n' && parity != 'o' && parity != 'e')
		parity = 'n';
	if (bits != '7' && bits != '8')
		bits = '8';

	printk("rtl838x_serial_init: cpu_freq: %d baud %d\n", cpu_freq, baud);
	memset(&p, 0, sizeof(p));
		
	p.type = PORT_16550A;
	p.membase = (unsigned char *) RTL838X_UART0_BASE;
	p.irq = RTL838X_UART0_EXT_IRQ;
	p.uartclk = ((cpu_freq << 1) /5) - baud * 24;
	p.flags = UPF_SKIP_TEST | UPF_LOW_LATENCY | UPF_SPD_CUST;
	p.iotype = UPIO_MEM;
	p.regshift = 2;
	p.fifosize = 1;
	p.custom_divisor = ((cpu_freq << 1) /5) / (baud * 16) - 1;
	
	/* Call early_serial_setup() here, to set up 8250 console driver */
	if (early_serial_setup(&p) != 0) {
		ret = 1;
	}
#endif
	return 0;
}
