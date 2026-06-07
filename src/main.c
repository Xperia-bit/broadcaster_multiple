/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include "print.h"

int broadcaster_multiple(void);

int main(void)
{
	df_print_init();
	int err;
	k_msleep(4000);

	df_print("Starting Multiple Broadcaster Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		df_print("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	(void)broadcaster_multiple();

	df_print("Exiting %s thread.\n", __func__);
	return 0;
}
