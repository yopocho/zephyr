/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>

// #include "zephyr/drivers/ieee802154/ieee802154_s2lp.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
// static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
const struct device *const ieee802154_dev = DEVICE_DT_GET(DT_NODELABEL(s2lp0));

int main(void)
{
	S2LP_Init(ieee802154_dev);
	// int ret;
	// bool led_state = true;

	// if (!gpio_is_ready_dt(&led)) {
	// 	return 0;
	// }

	// ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	// if (ret < 0) {
	// 	return 0;
	// }

	// while (1) {
	// 	ret = gpio_pin_toggle_dt(&led);
	// 	if (ret < 0) {
	// 		return 0;
	// 	}

	// 	led_state = !led_state;
	// 	printf("LED state: %s\n", led_state ? "ON" : "OFF");
	// 	k_msleep(SLEEP_TIME_MS);
	// }
	return 0;
}
